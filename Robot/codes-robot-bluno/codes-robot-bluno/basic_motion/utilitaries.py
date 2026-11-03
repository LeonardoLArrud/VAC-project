#########################################################################
# CENTRALESUPELEC : ST5 VAC challenge week
#########################################################################

try:
    import serial          # so existe no robo; assim o Webots tambem importa este arquivo
except ImportError:
    serial = None
import struct
import math
import csv
import numpy as np

def read_i16(f):
    return struct.unpack('<h', bytearray(f.read(2)))[0]

def read_i32(f):
    return struct.unpack('<l', bytearray(f.read(4)))[0]

def write_i16(f, value):
    f.write(struct.pack('<h', value))

def write_i32(f, value):
    f.write(struct.pack('<l', value))

def WaitACK(arduino):
    rep=b''
    while rep==b'':					# waiting acknowledgement of B2
        rep=arduino.readline()
    print(rep.decode())

def sendCmdi(arduino,cmd,arg1,arg2,arg3,arg4): # send command with 4 ints
    arduino.write(cmd)
    write_i16(arduino, arg1)
    write_i16(arduino, arg2)
    write_i16(arduino, arg3)
    write_i16(arduino, arg4)
    WaitACK(arduino)

def sendCmdl(arduino,cmd,arg1,arg2): # send command with 2 longs
    arduino.write(cmd)
    write_i32(arduino, arg1)
    write_i32(arduino, arg2)
    WaitACK(arduino)

def retrCmdi(arduino,cmd): # retrieve command with 4 ints
    arduino.write(cmd)
    val1=read_i16(arduino)
    val2=read_i16(arduino)
    val3=read_i16(arduino)
    val4=read_i16(arduino)
    return val1,val2,val3,val4
    WaitACK(arduino)

def retrCmdl(arduino,cmd): # retrieve command with 2 longs
    arduino.write(cmd)
    val1=read_i32(arduino)
    val2=read_i32(arduino)
    return val1,val2
    WaitACK(arduino)


#########################################################################
# SEGUIDOR DE LINHA (pure pursuit) - mesma logica do controlador Webots da equipe
#
# Nao conhece hardware: recebe a imagem em tons de cinza e devolve a velocidade
# de cada lado em m/s. Roda igual no robo (motorization_test.py) e no Webots (basique.py).
#########################################################################

class SeguidorLinha:
    def __init__(self, largura_px, altura_px, fov, v_roda_max,
                 h_cam=0.132, x_cam=0.104, tilt_graus=30.0,
                 distancias=(0.20, 0.26, 0.31, 0.36),
                 k=0.135, v0=0.13, tau_ef=0.10, a_lat=1.0, a_mais=0.5, a_menos=1.5,
                 w_busca=1.5, limiar=150, linha_clara=True, nu=480):
        """largura_px, altura_px, fov: camera (fov horizontal em rad).
        v_roda_max: velocidade maxima de uma roda em m/s.
        h_cam, x_cam, tilt_graus: altura da lente, distancia da lente ao centro do robo, inclinacao.
        limiar: nivel de cinza da linha (0-255) ou 'auto' (Otsu a cada imagem).
        linha_clara: True = linha clara em chao escuro (Webots), False = linha escura."""
        self.nu = nu
        self.nv = round(altura_px * nu / largura_px)
        self.f = (self.nu / 2) / math.tan(fov / 2)
        self.cu, self.cv = (self.nu - 1) / 2, (self.nv - 1) / 2
        self.k, self.v0, self.tau_ef = k, v0, tau_ef
        self.a_lat, self.a_mais, self.a_menos, self.w_busca = a_lat, a_mais, a_menos, w_busca
        self.v_roda_max = v_roda_max
        self.limiar, self.linha_clara = limiar, linha_clara
        self.d_esquecer = distancias[1]

        # faixas da imagem: (distancia no chao, linha da imagem, m por pixel, meia-largura vista)
        tilt = math.radians(tilt_graus)
        self.faixas = []
        for dj in distancias:
            q = math.tan(math.atan2(h_cam, dj - x_cam) - tilt)
            lin = round(self.cv + self.f * q)
            if 2 <= lin < self.nv - 2:
                den = math.sin(tilt) + q * math.cos(tilt)
                self.faixas.append((dj, lin, h_cam / (self.f * den), h_cam * math.tan(fov / 2) / den))
        if not self.faixas:
            raise ValueError("nenhuma faixa cai dentro da imagem: confira h_cam, x_cam e tilt")
        self.reset()

    def reset(self, manter_linha=False):
        """Zera a velocidade e esquece onde a linha estava (usar depois da virada do obstaculo).
        manter_linha=True: so zera v e w (o robo ficou parado sem girar, a linha continua onde estava)."""
        self.v = 0.0
        self.w = 0.0
        if manter_linha:
            return
        self.memoria = None
        self.andado = 0.0
        self.lado = 1.0
        self.u_prev = [self.cu] * len(self.faixas)

    def linha_a_frente(self, g):
        """Desvio lateral Y (m) da linha que o seguidor usaria como alvo, procurando a partir
        do centro da imagem e sem mudar o estado do seguidor. None se nao ve a linha.
        Usado durante a virada do obstaculo para saber quando a linha voltou a frente."""
        limiar = self._limiar_da_imagem(g)
        for j in reversed(range(len(self.faixas))):
            dj, lin, mj, wj = self.faixas[j]
            u = self.achar_linha(g, lin, self.cu, limiar)
            if u is None:
                continue
            y = -mj * (u - self.cu)
            if abs(y) <= 0.7 * wj:
                return y
        return None

    def _limiar_da_imagem(self, g):
        if self.limiar != 'auto':
            return self.limiar
        if g.std() < 8:                       # imagem uniforme: sem linha
            return 256 if self.linha_clara else -1
        import cv2
        t, _ = cv2.threshold(g.astype(np.uint8), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return t

    def achar_linha(self, g, lin, u_ref, limiar):
        faixa = g[lin - 2: lin + 3].mean(axis=0)
        linha = (faixa > limiar) if self.linha_clara else (faixa < limiar)
        borda = np.diff(np.concatenate(([0], linha.astype(np.int8), [0])))
        ini, fim = np.flatnonzero(borda == 1), np.flatnonzero(borda == -1)
        ok = fim - ini >= 2
        if not ok.any():
            return None
        centros = (ini[ok] + fim[ok] - 1) / 2
        return float(centros[np.argmin(np.abs(centros - u_ref))])

    def passo(self, g, dt):
        """g: imagem cinza (nv x nu). Devolve (v_esq, v_dir, texto do estado)."""
        limiar = self._limiar_da_imagem(g)
        alvo = None
        for j in reversed(range(len(self.faixas))):       # da faixa mais longe para a mais perto
            dj, lin, mj, wj = self.faixas[j]
            u = self.achar_linha(g, lin, self.u_prev[j], limiar)
            if u is None:
                continue
            self.u_prev[j] = u
            y = -mj * (u - self.cu)
            if abs(y) <= 0.7 * wj:
                alvo = (dj, y)
                break

        if alvo is not None:
            self.memoria = alvo
            self.andado = 0.0
        elif self.memoria is not None:                    # linha sumiu: segue o ultimo alvo por odometria
            dpsi = self.w * dt
            dx = self.v * dt * math.cos(dpsi / 2)
            dy = self.v * dt * math.sin(dpsi / 2)
            x0, y0 = self.memoria[0] - dx, self.memoria[1] - dy
            self.memoria = (math.cos(dpsi) * x0 + math.sin(dpsi) * y0,
                            -math.sin(dpsi) * x0 + math.cos(dpsi) * y0)
            self.andado += self.v * dt
            if self.memoria[0] <= 0.05 or self.andado > self.d_esquecer:
                self.memoria = None
            alvo = self.memoria

        if alvo is None:                                  # perdido: gira para o lado da ultima linha
            self.v = 0.0
            self.w = self.w_busca * self.lado
            return -self.k * self.w, self.k * self.w, "PERDIDO"

        L, Y = alvo
        self.lado = 1.0 if Y >= 0 else -1.0
        c = 2 * Y / (L * L + Y * Y)                       # curvatura do pure pursuit
        v_alvo = min(self.v0, 0.163 * L / self.tau_ef, math.sqrt(self.a_lat / max(abs(c), 1e-6)),
                     self.v_roda_max / (1 + self.k * abs(c)))
        self.v = min(max(v_alvo, self.v - self.a_menos * dt), self.v + self.a_mais * dt)
        self.w = self.v * c
        return (self.v - self.k * self.w, self.v + self.k * self.w,
                "L=%.2f Y=%+.3f c=%+.2f v=%.2f" % (L, Y, c, self.v))


def linha_voltou(y, y_antes, tol):
    """Durante a virada: a linha esta centrada a frente (|Y| <= tol) ou cruzou o centro
    entre duas imagens seguidas (virada rapida demais para pegar |Y| pequeno)."""
    if y is None:
        return False
    return abs(y) <= tol or (y_antes is not None and y * y_antes < 0)


def seguir_linha(robo, seg, duracao=None, seco=False, log=None, print_s=1.0, tol_virada=0.04):
    """Laco principal, igual no robo e no Webots.

    robo precisa ter: agora(), passo(), atualizar(), estado, distancia_cm, cinza(nu, nv),
    rodas(), parar() e fim_virada().
    O obstaculo e tratado pelo proprio robo (Arduino no robo real, Python no Webots):
      VERIFICANDO / SEM_SENSOR: o robo esta parado e o seguidor so espera.
      VIRANDO: o robo gira no eixo sozinho; aqui so se olha a camera e, quando a linha volta
        a frente (linha_voltou), chama robo.fim_virada(). O robo ignora o pedido antes do giro
        minimo (~100 graus), quando a camera ainda ve a linha de onde ele saiu, e para sozinho
        no giro maximo (~420 graus) se a linha nao aparecer.
      Na volta a ANDANDO: depois de virar, o seguidor recomeca do zero; depois de so parar
        (o obstaculo saiu da frente), so zera v e w.
    seco=True: calcula e imprime, mas nao manda nada aos motores (teste com o robo suspenso).
    log: nome de um arquivo CSV para registrar cada passo."""
    arq = open(log, 'w', newline='') if log else None
    escritor = csv.writer(arq) if arq else None
    if escritor:
        escritor.writerow(['t', 'estado_robo', 'dist_cm', 'v_esq', 'v_dir', 'v', 'w', 'seguidor'])
    t0 = antes = ultimo_print = robo.agora()
    pausado = False     # o robo saiu de ANDANDO (obstaculo)
    virou = False       # ... e passou por VIRANDO
    y_antes = None      # desvio da linha na imagem anterior, durante a virada
    try:
        while robo.passo():
            agora = robo.agora()
            dt, antes = agora - antes, agora
            if duracao is not None and agora - t0 >= duracao:
                break
            robo.atualizar()
            if robo.estado == 'VIRANDO':
                pausado = virou = True
                v_e = v_d = 0.0
                y = seg.linha_a_frente(robo.cinza(seg.nu, seg.nv))
                if linha_voltou(y, y_antes, tol_virada):
                    robo.fim_virada()
                    texto = "VIRANDO Y=%+.3f -> fim_virada" % y
                else:
                    texto = "VIRANDO sem linha" if y is None else "VIRANDO Y=%+.3f" % y
                y_antes = y
            elif robo.estado != 'ANDANDO':
                pausado = True
                y_antes = None
                texto, v_e, v_d = robo.estado, 0.0, 0.0
            else:
                if pausado:                     # acabou de sair do obstaculo
                    seg.reset(manter_linha=not virou)
                    pausado = virou = False
                y_antes = None
                g = robo.cinza(seg.nu, seg.nv)
                v_e, v_d, texto = seg.passo(g, dt)
                if not seco:
                    robo.rodas(v_e, v_d)
            if escritor:
                escritor.writerow(["%.3f" % (agora - t0), robo.estado, robo.distancia_cm,
                                   "%.3f" % v_e, "%.3f" % v_d, "%.3f" % seg.v, "%.3f" % seg.w, texto])
            if agora - ultimo_print >= print_s:
                extra = "  (seco: esq=%+.3f dir=%+.3f m/s)" % (v_e, v_d) if seco else ""
                print("[PP] t=%6.2fs d=%scm %s%s" % (agora - t0, robo.distancia_cm, texto, extra))
                ultimo_print = agora
    finally:
        robo.parar()
        if arq:
            arq.close()
