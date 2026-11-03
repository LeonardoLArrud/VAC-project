import math
import time
import numpy as np
from controller import Robot
from collections import deque


#configuracao sonar
THRESHOLD = 2000          # m: abaixo disso é obstáculo
TEMPO_VERIFICACAO = 2.0   # s (tempo simulados)

TIME_STEP = 64
H_CAM = 0.132
X_CAM = 0.104
TILT = math.radians(30)
D = [0.20, 0.26, 0.31, 0.36]
K = 0.135
R_RODA = 0.0325
V0 = 0.40
TAU_EF = 0.10
A_LAT = 1.0
A_MAIS = 0.5
A_MENOS = 1.5
W_BUSCA = 1.5
LIMIAR = 150
LARGURA = 480

robot = Robot()
dt = TIME_STEP / 1000.0
cam = robot.getDevice("PiCam")
cam.enable(TIME_STEP)
sensor = robot.getDevice("URM37")
sensor.enable(TIME_STEP)
dir_motores = [robot.getDevice(n) for n in ("Motor1", "Motor3")]
esq_motores = [robot.getDevice(n) for n in ("Motor2", "Motor4")]
for m in dir_motores + esq_motores:
    m.setPosition(float("inf"))
    m.setVelocity(0.0)
# sensor     
historico = deque(maxlen=3)
def ler_ultrassom():
    historico.append(sensor.getValue())
    return sorted(historico)[len(historico) // 2]

def parar_motores():
    for m in dir_motores + esq_motores:
        m.setVelocity(0.0)
def verificar_objeto():
    """Chame quando d <= THRESHOLD. Para, observa por 2 s (tempo simulado).
    Se o obstáculo continua lá, gira 180°. Devolve True se girou."""
    parar_motores()
    inicio = robot.getTime()
    while robot.getTime() - inicio < TEMPO_VERIFICACAO:
        if robot.step(TIME_STEP) == -1:
            return False
        if ler_ultrassom() > THRESHOLD:
            return False          # saiu da frente: não era estático
    girar_ate_linha()                 # gira até a câmera ver a linha de novo (em vez de girar por tempo)
    historico.clear()                 # leituras antigas (do obstáculo) não valem mais
    u_prev[:] = [CU] * len(faixas)    # esquece a posição da linha de antes da virada
    return True              


PHI_MAX = dir_motores[0].getMaxVelocity()
V_RODA_MAX = PHI_MAX * R_RODA

NU = LARGURA
NV = round(cam.getHeight() * LARGURA / cam.getWidth())
F = (NU / 2) / math.tan(cam.getFov() / 2)
CU, CV = (NU - 1) / 2, (NV - 1) / 2

faixas = []
for dj in D:
    q = math.tan(math.atan2(H_CAM, dj - X_CAM) - TILT)
    lin = round(CV + F * q)
    if 2 <= lin < NV - 2:
        den = math.sin(TILT) + q * math.cos(TILT)
        faixas.append((dj, lin, H_CAM / (F * den), H_CAM * math.tan(cam.getFov() / 2) / den))
u_prev = [CU] * len(faixas)


def cinza():
    img = np.frombuffer(cam.getImage(), np.uint8).reshape((cam.getHeight(), cam.getWidth(), 4))
    passo = cam.getWidth() // LARGURA
    return img[::passo, ::passo, :3].mean(axis=2)[:NV, :NU]


def achar_linha(g, lin, u_ref):
    branco = (g[lin - 2 : lin + 3].mean(axis=0) > LIMIAR).astype(np.int8)
    borda = np.diff(np.concatenate(([0], branco, [0])))
    ini, fim = np.flatnonzero(borda == 1), np.flatnonzero(borda == -1)
    ok = fim - ini >= 2
    if not ok.any():
        return None
    centros = (ini[ok] + fim[ok] - 1) / 2
    return float(centros[np.argmin(np.abs(centros - u_ref))])


def rodas(v_e, v_d):
    for m in esq_motores:
        m.setVelocity(float(np.clip(v_e / R_RODA, -PHI_MAX, PHI_MAX)))
    for m in dir_motores:
        m.setVelocity(float(np.clip(v_d / R_RODA, -PHI_MAX, PHI_MAX)))
def girar_no_eixo(vel_ang=W_BUSCA):
    """Gira no próprio eixo. vel_ang em rad/s do robô: positivo = esquerda, negativo = direita.
    Chame a cada passo do loop, para girar continuamente."""
    rodas(-K * vel_ang, K * vel_ang)


def girar_angulo(graus, vel_ang=W_BUSCA):
    """Gira um ângulo e para. Positivo = esquerda, negativo = direita.
    Bloqueia até terminar (a câmera não é processada durante a virada)."""
    sentido = 1.0 if graus >= 0 else -1.0
    alvo = math.radians(abs(graus))
    girado = 0.0
    while girado < alvo:
        girar_no_eixo(sentido * abs(vel_ang))
        if robot.step(TIME_STEP) == -1:
            break
        girado += abs(vel_ang) * dt
    parar_motores()

def procurar_alvo(g):
    """Mesma busca do loop principal, mas sem alterar u_prev. Devolve (dj, y) ou None."""
    for j in reversed(range(len(faixas))):
        dj, lin, mj, wj = faixas[j]
        u = achar_linha(g, lin, CU)
        if u is None:
            continue
        y = -mj * (u - CU)
        if abs(y) <= 0.7 * wj:
            return (dj, y)
    return None


def girar_ate_linha(graus_min=100, graus_max=420, vel_ang=1.0, tol_y=0.04):
    """Vira no eixo (para a esquerda) até a câmera ver a linha de novo, centrada à frente.
    Gira pelo menos graus_min, para não parar na própria linha de onde saiu, e no máximo
    graus_max (maior que 360 de propósito: as rodas derrapam e giram menos que o calculado).
    Devolve True se reencontrou a linha."""
    minimo = math.radians(graus_min)
    maximo = math.radians(graus_max)
    girado = 0.0
    achou = False
    while girado < maximo:
        girar_no_eixo(abs(vel_ang))
        if robot.step(TIME_STEP) == -1:
            break
        girado += abs(vel_ang) * dt
        if girado >= minimo:
            alvo = procurar_alvo(cinza())
            if alvo is not None and abs(alvo[1]) <= tol_y:
                achou = True
                break
    parar_motores()
    return achou


v = 0.0
w = 0.0
memoria = None
andado = 0.0
lado = 1.0
ultimo_print = 0.0

while robot.step(TIME_STEP) != -1:
    g = cinza()
    alvo = None
    d = ler_ultrassom()
    if d <= THRESHOLD:
        virou = verificar_objeto()
        v = w = 0.0                   # o robô ficou parado, então o estado interno também zera
        if virou:
            memoria = None
            andado = 0.0              # (antes estava 'andando', variável que não existe)
            continue
    for j in reversed(range(len(faixas))):
        dj, lin, mj, wj = faixas[j]
        u = achar_linha(g, lin, u_prev[j])
        if u is None:
            continue
        u_prev[j] = u
        y = -mj * (u - CU)
        if abs(y) <= 0.7 * wj:
            alvo = (dj, y)
            break

    if alvo is not None:
        memoria = alvo
        andado = 0.0
    elif memoria is not None:
        dpsi = w * dt
        dx = v * dt * math.cos(dpsi / 2)
        dy = v * dt * math.sin(dpsi / 2)
        x0, y0 = memoria[0] - dx, memoria[1] - dy
        memoria = (math.cos(dpsi) * x0 + math.sin(dpsi) * y0, -math.sin(dpsi) * x0 + math.cos(dpsi) * y0)
        andado += v * dt
        if memoria[0] <= 0.05 or andado > D[1]:
            memoria = None
        alvo = memoria

    if alvo is None:
        v = 0.0
        w = W_BUSCA * lado
        rodas(-K * w, K * w)
        estado = "PERDIDO"
    else:
        L, Y = alvo
        lado = 1.0 if Y >= 0 else -1.0
        c = 2 * Y / (L * L + Y * Y)
        v_alvo = min(V0, 0.163 * L / TAU_EF, math.sqrt(A_LAT / max(abs(c), 1e-6)), V_RODA_MAX / (1 + K * abs(c)))
        v = min(max(v_alvo, v - A_MENOS * dt), v + A_MAIS * dt)
        w = v * c
        rodas(v - K * w, v + K * w)
        estado = f"L={L:.2f} Y={Y:+.3f} c={c:+.2f} v={v:.2f}"

    t = robot.getTime()
    if t - ultimo_print >= 1.0:
        print(f"[PP] t={t:6.2f}s {estado}")
        ultimo_print = t