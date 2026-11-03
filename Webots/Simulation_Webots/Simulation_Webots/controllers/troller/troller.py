# Controlador Webots do robo Troller com a MESMA logica do robo real.
#
# Seguidor de linha (SeguidorLinha) e laco principal (seguir_linha) vem de
# Robot/codes-robot-bluno/codes-robot-bluno/basic_motion/utilitaries.py, o mesmo arquivo
# que roda na Raspberry. Aqui so fica o "robo": motores, camera, sonar e a maquina de
# estados do obstaculo, que no robo real esta no Arduino (robo_pista.ino):
#   ANDANDO -> VERIFICANDO (parado 2 s) -> BLOQUEADO -> VIRANDO -> ANDANDO
# Na virada o robo gira para a esquerda ate a camera ver a linha a frente (fim_virada,
# chamado por seguir_linha), com giro minimo de 100 graus e maximo de 420 graus,
# como o girar_ate_linha() do basique.py.
#
# Para usar: no Webots, no no Cherokey_4W_v3, troque o campo controller para "troller"
# (ou abra worlds/huit_troller.wbt, que ja vem assim e tem uma caixa na pista).
# Opcional: a variavel de ambiente TROLLER_LOG=<arquivo> grava as mensagens num arquivo.

import os
import sys

if os.environ.get('TROLLER_LOG'):
    sys.stdout = sys.stderr = open(os.environ['TROLLER_LOG'], 'w', buffering=1)

import math
from collections import deque

import numpy as np
from controller import Robot

# pasta basic_motion do codigo do robo (raiz do repositorio: 5 niveis acima desta pasta)
# (abspath tira os '..': sem isso o Windows recusa caminhos com mais de 260 caracteres)
AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.abspath(os.path.join(AQUI, '..', '..', '..', '..', '..'))
sys.path.append(os.path.join(RAIZ, 'Robot', 'codes-robot-bluno', 'codes-robot-bluno', 'basic_motion'))
from utilitaries import SeguidorLinha, seguir_linha

TIME_STEP = 64
R_RODA = 0.0325             # m: raio da roda do modelo
K = 0.135                   # m: meia-bitola efetiva
V0 = 0.40                   # m/s em reta (o mesmo do basique.py)
THRESHOLD_CM = 30           # igual ao THRESHOLD do robo_pista.ino
TEMPO_VERIFICACAO = 2.0     # s, igual ao TEMPO_VERIFICACAO do robo_pista.ino
W_GIRO = 1.0                # rad/s na virada (positivo = esquerda), como no basique.py
GRAUS_MIN_VIRADA = 100      # iguais aos do robo_pista.ino
GRAUS_MAX_VIRADA = 420


class RoboWebots:
    """Mesma interface do RoboReal (motorization_test.py)."""

    def __init__(self):
        self.robot = Robot()
        self.dt = TIME_STEP / 1000.0
        self.cam = self.robot.getDevice("PiCam")
        self.cam.enable(TIME_STEP)
        self.sonar = self.robot.getDevice("URM37")
        self.sonar.enable(TIME_STEP)
        self.dir_motores = [self.robot.getDevice(n) for n in ("Motor1", "Motor3")]   # frente e tras direita
        self.esq_motores = [self.robot.getDevice(n) for n in ("Motor2", "Motor4")]   # frente e tras esquerda
        for m in self.dir_motores + self.esq_motores:
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
        self.phi_max = self.dir_motores[0].getMaxVelocity()
        self.v_max = self.phi_max * R_RODA
        self.largura_px, self.altura_px = self.cam.getWidth(), self.cam.getHeight()
        self.fov = self.cam.getFov()
        self.estado = 'ANDANDO'
        self.distancia_cm = -2
        self.historico = deque(maxlen=3)
        self.t_estado = 0.0
        self.girado = 0.0           # rad girados na virada atual (pelo comando; as rodas derrapam)

    def agora(self):
        return self.robot.getTime()

    def passo(self):
        return self.robot.step(TIME_STEP) != -1

    def _rodas(self, v_esq, v_dir):
        for m in self.esq_motores:
            m.setVelocity(float(np.clip(v_esq / R_RODA, -self.phi_max, self.phi_max)))
        for m in self.dir_motores:
            m.setVelocity(float(np.clip(v_dir / R_RODA, -self.phi_max, self.phi_max)))

    def rodas(self, v_esq, v_dir):
        if self.estado == 'ANDANDO':        # como no Arduino: durante o obstaculo o seguidor nao manda
            self._rodas(v_esq, v_dir)

    def parar(self):
        self._rodas(0.0, 0.0)

    def _mudar(self, novo):
        self.estado = novo
        self.t_estado = self.agora()
        print("E," + novo)

    def _terminar_virada(self, motivo):
        self.parar()
        print("VIROU,%d,%s  (%.0f graus pelo comando)"
              % (round((self.agora() - self.t_estado) * 1000), motivo, math.degrees(self.girado)))
        self._mudar('ANDANDO')

    def fim_virada(self):
        """A camera viu a linha a frente. Como no Arduino, so vale depois do giro minimo."""
        if self.estado == 'VIRANDO' and self.girado >= math.radians(GRAUS_MIN_VIRADA):
            self._terminar_virada('LINHA')

    def atualizar(self):
        """Ultrassom (mediana de 3) + maquina de estados do obstaculo, como no robo_pista.ino."""
        self.historico.append(self.sonar.getValue() / 50.0)     # o URM37.proto devolve us: 50 us = 1 cm
        d = sorted(self.historico)[len(self.historico) // 2]
        self.distancia_cm = int(d)
        if self.estado == 'ANDANDO':
            if d <= THRESHOLD_CM:
                self.parar()
                self._mudar('VERIFICANDO')
        elif self.estado == 'VERIFICANDO':
            if d > THRESHOLD_CM:
                self._mudar('ANDANDO')                          # saiu da frente: nao era estatico
            elif self.agora() - self.t_estado >= TEMPO_VERIFICACAO:
                print("BLOQUEADO")
                self.girado = 0.0
                self._mudar('VIRANDO')
                self._rodas(-K * W_GIRO, K * W_GIRO)            # gira no eixo para a esquerda
        elif self.estado == 'VIRANDO':
            self.girado += W_GIRO * self.dt                     # o passo que acabou de rodar girou isso
            if self.girado >= math.radians(GRAUS_MAX_VIRADA):
                self._terminar_virada('TEMPO')                  # a linha nao apareceu

    def cinza(self, nu, nv):
        img = np.frombuffer(self.cam.getImage(), np.uint8).reshape((self.altura_px, self.largura_px, 4))
        try:
            import cv2
            g = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
            return cv2.resize(g, (nu, nv), interpolation=cv2.INTER_AREA).astype(np.float32)
        except ImportError:                 # sem OpenCV: amostra 1 pixel a cada 'passo', como o basique.py
            passo = self.largura_px // nu
            return img[::passo, ::passo, :3].mean(axis=2)[:nv, :nu].astype(np.float32)


robo = RoboWebots()
seg = SeguidorLinha(robo.largura_px, robo.altura_px, robo.fov, robo.v_max,
                    k=K, v0=V0, limiar=150, linha_clara=True)   # no Webots a linha e branca no chao preto
print("Seguidor: %d faixas, imagem %dx%d" % (len(seg.faixas), seg.nu, seg.nv))
seguir_linha(robo, seg)
