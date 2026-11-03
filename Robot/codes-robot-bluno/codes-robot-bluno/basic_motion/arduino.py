#########################################################################
# CENTRALESUPELEC : ST5 VAC challenge week
#########################################################################

import serial 
import time
import numpy as np
from utilitaries import *


class arduino:

    def __init__(self,portnr):
        self.serialport = serial.Serial(port=portnr, baudrate=115200, timeout=0.1)
        
    #######################################
    #   connection to Arduino
    #######################################
    def		connect(self):
        if self.serialport.is_open==False:
            self.serialport.open()
        rep=' '   # emptying serial link
        while rep!=b'':
          rep = self.serialport.readline()
        print ("Connecting to Arduino")

        time.sleep(2)			# waiting for 2s for the Arduino to be initialised

        self.serialport.write(b'A22')		# requesting connection with simple 'OK' acknowledgement
        rep = self.serialport.readline()
        if rep.split()[0]==b'OK':
            self.serialport.write(b'I0')
            WaitACK(self.serialport)
            print(rep.decode())
            return True
        else:
            return False
  
        
    #######################################
    #   disconnection
    #######################################
    def		disconnect(self):
        self.serialport.write(b'a')	# disconnection
        self.serialport.close()         # closing serial link

    #######################################
    #   stop motors
    #######################################
    def  Stop(self):
        sendCmdi(self.serialport,b'C',0,0,0,0)

    #######################################
    #   moving forward
    #######################################
    def  Advance(self,c1,c2):
        sendCmdi(self.serialport,b'C',c1,c2,c1,c2)

    #######################################
    #   mise en route des moteurs en sens inverse 
    #######################################
    def  Back(self,c1,c2):
        sendCmdi(self.serialport,b'C',-c1,-c2,-c1,-c2)

    #######################################################
    #   mise en route des moteurs pour tourner à gauche
    #######################################################
    def  TurnLeft(self,c1,c2):
        sendCmdi(self.serialport,b'C',c1,-c2,c1,-c2)

    #######################################################
    #   mise en route des moteurs pour tourner à droite
    #######################################################
    def  TurnRight(self,c1,c2):
        sendCmdi(self.serialport,b'C',-c1,c2,-c1,c2)

    #######################################################
    #   mise en route de la protection infra rouge
    #######################################################
    def  IrOn(self):
        self.serialport.write(b'I1')
        WaitACK(self.serialport)

    #######################################################
    #   recuperation de la vitesse des moteurs
    #######################################################
    def  GetSpeed(self):
        vit1,vit2,vit3,vit4 = retrCmdi(self.serialport,b'P')
        return vit1,vit2,vit3,vit4


#########################################################################
#   Cliente do sketch robo_pista.ino (protocolo texto, 115200 bauds)
#
#   Comandos  : M<esq>,<dir>  (PWM -255..255)   F   G   X   T   V<n>   R<ms>
#   Mensagens : D,<cm>,<estado> a cada 500 ms   E,<estado>   BLOQUEADO   SEM_SENSOR
#               VIROU,<ms>,<LINHA|TEMPO>
#   O obstaculo (parar, verificar 2 s, virar) e tratado pelo proprio Arduino;
#   aqui acompanhamos o estado que ele avisa e, durante a virada, mandamos F
#   quando a camera ve a linha a frente.
#########################################################################

class ArduinoPista:

    def __init__(self, portnr='/dev/ttyACM0'):
        self.serialport = serial.Serial(port=portnr, baudrate=115200, timeout=0)  # leitura nao bloqueante
        self.estado = 'ANDANDO'     # ultimo estado avisado pelo Arduino
        self.dist = -2              # ultima distancia do diagnostico D, em cm (-1 invalida, -2 sem sinal)
        self.bloqueios = 0          # quantas vezes o Arduino avisou BLOQUEADO
        self.ultima_virada = None   # (ms, 'LINHA' ou 'TEMPO') da ultima mensagem VIROU
        self.mensagens = []         # ultimas linhas recebidas (uteis nos testes)
        self._buf = b''

    def connect(self):
        time.sleep(2)                       # a Bluno reinicia quando a porta serial abre
        self.serialport.reset_input_buffer()
        self.comando('X')                   # parado ate o primeiro M
        t0 = time.time()
        while time.time() - t0 < 1.5:       # o diagnostico D chega a cada 500 ms
            self.atualizar()
            if self.mensagens:
                print("Arduino conectado:", self.mensagens[-1])
                return True
            time.sleep(0.05)
        return False

    def comando(self, texto):
        self.serialport.write((texto + '\n').encode())

    def Motores(self, esq, dir_):
        esq = int(max(-255, min(255, esq)))
        dir_ = int(max(-255, min(255, dir_)))
        self.comando('M%d,%d' % (esq, dir_))

    def Stop(self):
        self.Motores(0, 0)

    def FimVirada(self):
        """A camera viu a linha a frente: o Arduino termina a virada (se ja girou o minimo)."""
        self.comando('F')

    def atualizar(self):
        """Le, sem bloquear, todas as linhas que o Arduino mandou desde a ultima chamada."""
        n = self.serialport.in_waiting
        if n:
            self._buf += self.serialport.read(n)
        while b'\n' in self._buf:
            linha, self._buf = self._buf.split(b'\n', 1)
            self._tratar(linha.decode(errors='ignore').strip())

    def _tratar(self, linha):
        if not linha:
            return
        self.mensagens = (self.mensagens + [linha])[-50:]
        if linha.startswith('D,'):
            partes = linha.split(',')
            if len(partes) >= 3:
                try:
                    self.dist = int(partes[1])
                except ValueError:
                    pass
                self.estado = partes[2]
        elif linha.startswith('E,'):
            self.estado = linha[2:]
        elif linha == 'BLOQUEADO':
            self.bloqueios += 1
        elif linha == 'SEM_SENSOR':
            self.estado = 'SEM_SENSOR'
        elif linha.startswith('VIROU,'):
            partes = linha.split(',')
            if len(partes) >= 3 and partes[1].isdigit():
                self.ultima_virada = (int(partes[1]), partes[2])

    def disconnect(self):
        self.comando('X')
        time.sleep(0.05)
        self.serialport.close()