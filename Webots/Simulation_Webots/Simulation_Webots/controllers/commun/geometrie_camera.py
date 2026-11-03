"""Modelo geometrico da camera: pixel <-> ponto no chao (referencial do robo).

Mesmo arquivo na simulacao e no robo real. So os parametros mudam
(altura, inclinacao, campo de visao, resolucao), lidos de calibracao.

Referencial do robo: x para frente, y para a esquerda, z para cima,
origem no centro do chassi projetado no chao.
Imagem: u para a direita, v para baixo, origem no canto superior esquerdo.
Modelo pinhole sem distorcao (exato no Webots; a verificar na camera real).
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class CameraModel:
    altura_m: float        # altura do centro optico acima do chao
    x_cam_m: float         # posicao da camera a frente do centro do chassi
    inclinacao_rad: float  # positivo = olhando para baixo
    hfov_rad: float        # campo de visao horizontal
    largura_px: int
    altura_px: int

    @property
    def f_px(self) -> float:
        return (self.largura_px / 2.0) / math.tan(self.hfov_rad / 2.0)

    @property
    def cx(self) -> float:
        return (self.largura_px - 1) / 2.0

    @property
    def cy(self) -> float:
        return (self.altura_px - 1) / 2.0

    def escalar(self, fator: float) -> "CameraModel":
        """Mesmo modelo para uma imagem reduzida (ex.: processar em 1/4)."""
        return CameraModel(self.altura_m, self.x_cam_m, self.inclinacao_rad,
                           self.hfov_rad, round(self.largura_px * fator),
                           round(self.altura_px * fator))

    def pixel_para_chao(self, u, v):
        """Retorna (x, y) no chao. NaN para pixels acima do horizonte."""
        u = np.asarray(u, dtype=float)
        v = np.asarray(v, dtype=float)
        # direcao do raio: frente, esquerda, cima (antes da inclinacao)
        frente = np.ones_like(u)
        esquerda = -(u - self.cx) / self.f_px
        cima = -(v - self.cy) / self.f_px
        c, s = math.cos(self.inclinacao_rad), math.sin(self.inclinacao_rad)
        frente_r = frente * c + cima * s
        cima_r = -frente * s + cima * c
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(cima_r < 0, self.altura_m / -cima_r, np.nan)
        return self.x_cam_m + t * frente_r, t * esquerda

    def chao_para_pixel(self, x, y):
        """Inverso de pixel_para_chao (ponto no chao -> pixel)."""
        x = np.asarray(x, dtype=float) - self.x_cam_m
        y = np.asarray(y, dtype=float)
        z = -self.altura_m
        c, s = math.cos(self.inclinacao_rad), math.sin(self.inclinacao_rad)
        frente = x * c - z * s
        cima = x * s + z * c
        with np.errstate(divide="ignore", invalid="ignore"):
            u = self.cx - self.f_px * y / frente
            v = self.cy - self.f_px * cima / frente
        return u, v
