#########################################################################
# CENTRALESUPELEC : ST5 VAC challenge week
#
# Programa principal no robo real: seguidor de linha (pure pursuit do Webots)
# com o Arduino programado com robo_pista.ino (obstaculo tratado no Arduino).
# Durante a virada do obstaculo, a Raspberry olha a camera e manda F ao Arduino
# quando a linha volta a frente (mesma regra do Webots).
#
# Uso, na Raspberry (pelo VNC):   python3 motorization_test.py
# Abre um menu com os testes de calibracao e o modo pista. Ctrl+C para o robo
# e volta ao menu.
#
# MotorizationTest() no fim do arquivo e o teste original da disciplina:
# so funciona com o firmware serial_link.ino carregado.
#########################################################################

import math
import time
import numpy as np
import cv2
from utilitaries import *
from arduino import arduino, ArduinoPista


# ---------- parametros (MEDIR / CALIBRAR no robo) ----------
PORTA = '/dev/ttyACM0'
LARGURA_CAM, ALTURA_CAM = 640, 480
FOV_CAM = math.radians(62.2)    # Pi Camera V2.1, campo horizontal (igual ao PiCamera.proto)
H_CAM = 0.132                   # m: altura da lente ao chao                     (MEDIR)
X_CAM = 0.104                   # m: da lente ao centro entre as rodas           (MEDIR)
TILT_GRAUS = 30.0               # graus: inclinacao da camera para baixo         (MEDIR)
K = 0.135                       # m: meia-bitola efetiva                         (teste 4)
PWM_MIN = 45                    # zona morta: abaixo disso o robo nao anda       (teste 3)
V_MAX = 0.45                    # m/s de uma roda com PWM 255                    (teste 3)
V0 = 0.08                       # m/s em reta: comecar devagar (o Webots usa 0.13)
LINHA_CLARA = True              # True = fita clara em chao escuro              (CONFERIR)
LIMIAR = 'auto'                 # 'auto' (Otsu a cada imagem) ou um numero 0-255
T_PASSO = 0.064                 # s: mesmo periodo do Webots


class Camera:
    """Pi Camera pelo Picamera2 (campo de visao completo); se faltar, cv2.VideoCapture."""

    def __init__(self, largura, altura):
        self.picam = None
        self.cap = None
        try:
            from picamera2 import Picamera2
            self.picam = Picamera2()
            main = {"size": (largura, altura), "format": "RGB888"}
            try:
                # raw 1640x1232 = modo do sensor com o campo de visao inteiro (4:3)
                cfg = self.picam.create_preview_configuration(main=main, raw={"size": (1640, 1232)})
            except Exception:
                cfg = self.picam.create_preview_configuration(main=main)
            self.picam.configure(cfg)
            self.picam.start()
            time.sleep(1.0)             # deixa a exposicao automatica estabilizar
        except ImportError:
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, largura)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, altura)

    def ler(self):
        """Imagem BGR (o RGB888 do Picamera2 ja vem na ordem BGR do OpenCV)."""
        if self.picam is not None:
            return self.picam.capture_array()
        ok, img = self.cap.read()
        if not ok:
            raise RuntimeError("a camera nao devolveu imagem")
        return img

    def fechar(self):
        if self.picam is not None:
            self.picam.stop()
        if self.cap is not None:
            self.cap.release()


class RoboReal:
    """Mesma interface que o RoboWebots do basique.py, para o seguir_linha() de utilitaries.py."""

    def __init__(self):
        self.ard = ArduinoPista(PORTA)
        if not self.ard.connect():
            raise RuntimeError("o Arduino nao respondeu: o robo_pista.ino esta carregado?")
        self.cam = Camera(LARGURA_CAM, ALTURA_CAM)
        self.largura_px, self.altura_px, self.fov = LARGURA_CAM, ALTURA_CAM, FOV_CAM
        self.v_max = V_MAX
        self.estado = 'ANDANDO'
        self._prox = time.monotonic()

    @property
    def distancia_cm(self):
        return self.ard.dist

    def agora(self):
        return time.monotonic()

    def passo(self):
        """Espera o fim do periodo T_PASSO (como o robot.step do Webots)."""
        self._prox += T_PASSO
        espera = self._prox - time.monotonic()
        if espera > 0:
            time.sleep(espera)
        else:
            self._prox = time.monotonic()   # atrasado: nao tenta recuperar em rajada
        return True

    def atualizar(self):
        self.ard.atualizar()
        self.estado = self.ard.estado

    @staticmethod
    def pwm(v):
        """m/s -> PWM, com a zona morta dos motores."""
        if abs(v) < 1e-3:
            return 0
        p = PWM_MIN + (255 - PWM_MIN) * min(abs(v) / V_MAX, 1.0)
        return int(math.copysign(p, v))

    def rodas(self, v_esq, v_dir):
        self.ard.Motores(self.pwm(v_esq), self.pwm(v_dir))

    def parar(self):
        self.ard.Stop()

    def fim_virada(self):
        self.ard.FimVirada()

    def cinza(self, nu, nv):
        g = cv2.cvtColor(self.cam.ler(), cv2.COLOR_BGR2GRAY)
        return cv2.resize(g, (nu, nv), interpolation=cv2.INTER_AREA).astype(np.float32)

    def fechar(self):
        try:
            self.parar()
            self.ard.disconnect()
        finally:
            self.cam.fechar()


def novo_seguidor(robo):
    return SeguidorLinha(robo.largura_px, robo.altura_px, robo.fov, robo.v_max,
                         h_cam=H_CAM, x_cam=X_CAM, tilt_graus=TILT_GRAUS, k=K, v0=V0,
                         limiar=LIMIAR, linha_clara=LINHA_CLARA)


def ler_numero(pergunta, padrao=None):
    while True:
        txt = input(pergunta).strip().replace(',', '.')
        if txt == '' and padrao is not None:
            return padrao
        try:
            return float(txt)
        except ValueError:
            print("  digite um numero")


#########################################################################
# TESTES DE CALIBRACAO (fazer nesta ordem na primeira vez)
#########################################################################

def teste_camera(robo, seg):
    """1. Geometria: salva teste_camera.png com as faixas onde o seguidor procura a linha.
    Coloque fitas no chao a 20, 26, 31 e 36 cm do centro do robo: elas devem cair nas linhas."""
    img = cv2.resize(robo.cam.ler(), (seg.nu, seg.nv), interpolation=cv2.INTER_AREA)
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    limiar = seg._limiar_da_imagem(g)
    print("limiar usado: %s" % limiar)
    for dj, lin, mj, wj in seg.faixas:
        u = seg.achar_linha(g, lin, seg.cu, limiar)
        cv2.line(img, (0, lin), (seg.nu - 1, lin), (0, 0, 255), 1)
        cv2.putText(img, "%d cm" % round(dj * 100), (4, lin - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        if u is None:
            print("  faixa %2d cm (linha %3d da imagem): linha NAO encontrada" % (round(dj * 100), lin))
        else:
            cv2.circle(img, (int(u), lin), 5, (0, 255, 0), 2)
            print("  faixa %2d cm (linha %3d da imagem): linha em u=%5.1f px, desvio lateral %+.3f m"
                  % (round(dj * 100), lin, u, -mj * (u - seg.cu)))
    cv2.imwrite("teste_camera.png", img)
    print("imagem salva em teste_camera.png")


def teste_sensor(robo, duracao=10.0):
    """2. Ultrassom: mostra a distancia que o Arduino mede (robo parado)."""
    robo.parar()
    t0 = time.monotonic()
    while time.monotonic() - t0 < duracao:
        robo.atualizar()
        print("  distancia = %4s cm   estado = %s" % (robo.distancia_cm, robo.estado))
        time.sleep(0.5)


def teste_motores(robo):
    """3. Velocidade x PWM: anda 2 s em linha reta com varios PWM; voce mede a distancia."""
    pwms, vels = [], []
    for p in (60, 90, 120, 150):
        input("\nPWM %d: coloque o robo no inicio, sem nada na frente, e aperte Enter" % p)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            robo.ard.Motores(p, p)          # repetir: o Arduino para se nao receber M por 300 ms
            robo.atualizar()
            time.sleep(0.1)
        robo.parar()
        d = ler_numero("  distancia percorrida em cm: ")
        pwms.append(p)
        vels.append(d / 100.0 / 2.0)
        print("  -> %.3f m/s" % vels[-1])
    a, b = np.polyfit(pwms, vels, 1)
    if a <= 0:
        print("os dados nao mostram velocidade crescendo com o PWM: refaca o teste")
        return
    print("\nAjuste linear: v = %.5f * PWM %+.4f" % (a, b))
    print("Sugestao para o topo do arquivo: PWM_MIN = %d   V_MAX = %.2f" % (max(0, round(-b / a)), a * 255 + b))


def teste_giro_k(robo, w=1.0, duracao=3.0):
    """4. Meia-bitola efetiva K: gira no lugar pedindo w rad/s; voce mede o angulo girado."""
    input("\nO robo vai girar no lugar por %.0f s (esperado: %.0f graus). Enter para comecar"
          % (duracao, math.degrees(w * duracao)))
    t0 = time.monotonic()
    while time.monotonic() - t0 < duracao:
        robo.rodas(-K * w, K * w)
        robo.atualizar()
        time.sleep(0.05)
    robo.parar()
    graus = ler_numero("  angulo realmente girado, em graus: ")
    if graus > 0:
        print("Sugestao: K = %.3f   (giro menor que o esperado -> K maior)"
              % (K * math.degrees(w * duracao) / graus))


def teste_virada_arduino(robo):
    """5. Virada de 180 graus do Arduino (comando T, duracao tempoVirar do sketch).
    Na pista a virada termina quando a camera ve a linha (comando F), mas o giro minimo
    (~100 graus) e o maximo (~420 graus) do Arduino sao calculados a partir de tempoVirar."""
    tempo = ler_numero("tempoVirar atual no sketch, em ms [1200]: ", 1200)
    robo.ard.comando('R%d' % tempo)
    input("O robo vai girar com o comando T. Enter para comecar")
    robo.ard.comando('T')
    time.sleep(tempo / 1000.0 + 0.5)
    graus = ler_numero("  angulo girado, em graus: ")
    if graus > 0:
        novo = round(tempo * 180.0 / graus)
        robo.ard.comando('R%d' % novo)
        print("Mandei R%d para o Arduino. Para ficar salvo, troque tempoVirar = %d no robo_pista.ino" % (novo, novo))


def teste_seco(robo, seg, duracao=20.0):
    """6. Sentido das correcoes, com o robo SUSPENSO: calcula e imprime, sem mexer os motores.
    Linha a direita da imagem -> esq maior que dir."""
    seguir_linha(robo, seg, duracao=duracao, seco=True, print_s=0.5)


def pista(robo, seg):
    """7. Seguir a linha na pista (Ctrl+C para parar). Registra tudo em um CSV."""
    nome = time.strftime("log_pista_%Hh%M.csv")
    print("seguindo a linha; registro em %s; Ctrl+C para parar" % nome)
    seg.reset()
    try:
        seguir_linha(robo, seg, log=nome)
    finally:
        print("obstaculos (BLOQUEADO): %d   ultima virada: %s" % (robo.ard.bloqueios, robo.ard.ultima_virada))


#########################################################################
# Menu
#########################################################################

def menu():
    robo = RoboReal()
    seg = novo_seguidor(robo)
    print("Seguidor: %d faixas, imagem %dx%d" % (len(seg.faixas), seg.nu, seg.nv))
    opcoes = [
        ("Camera e geometria (salva teste_camera.png)", lambda: teste_camera(robo, seg)),
        ("Ultrassom (10 s, robo parado)", lambda: teste_sensor(robo)),
        ("Velocidade x PWM (PWM_MIN, V_MAX)", lambda: teste_motores(robo)),
        ("Meia-bitola K (giro no lugar)", lambda: teste_giro_k(robo)),
        ("Virada de 180 graus do Arduino (tempoVirar)", lambda: teste_virada_arduino(robo)),
        ("Seguidor sem motores (robo suspenso)", lambda: teste_seco(robo, seg)),
        ("Seguir a linha na pista", lambda: pista(robo, seg)),
    ]
    try:
        while True:
            print("\n===== Robo Troller =====")
            for i, (nome, _) in enumerate(opcoes, 1):
                print("  %d. %s" % (i, nome))
            print("  0. Sair")
            escolha = input("> ").strip()
            if escolha == '0':
                break
            if not escolha.isdigit() or not 1 <= int(escolha) <= len(opcoes):
                continue
            try:
                opcoes[int(escolha) - 1][1]()
            except KeyboardInterrupt:
                print("\ninterrompido")
            finally:
                robo.parar()
    finally:
        robo.fechar()
        print("End of program")


#########################################################################
# Teste original da disciplina (so com o firmware serial_link.ino)
#########################################################################

def MotorizationTest(car):

    steptime = 2 			# duration of each stage

    if (1==1):
        print("Moving forward")
        car.Advance(150,150)
        time.sleep(1*steptime)
        car.Stop()
        time.sleep(1)

    if (1==1):
        print(" moving backward")
        car.Back(150,150)
        time.sleep(steptime)
        car.Stop()
        time.sleep(1)

    if (1==1):
        print(" turning left")
        car.TurnLeft(120,120)
        time.sleep(steptime)
        car.Stop()
        time.sleep(1)

        print(" turning right")
        car.TurnRight(120,120)
        time.sleep(steptime)
        car.Stop()
        time.sleep(1)


if __name__ == "__main__":
    menu()
