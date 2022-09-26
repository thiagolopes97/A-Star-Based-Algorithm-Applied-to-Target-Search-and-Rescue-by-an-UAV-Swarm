from constants import *
import pygame as pg
from math import atan2, pi, exp, floor
import random
import copy 
import numpy as np
vec = pg.math.Vector2 

def normalFunction(omega,center,position):
    f = exp( -omega*((position.x - center.x) + (position.y - center.y)))
    return f

def bivariateFunction(alpha,beta,center,position):
    '''
        Calculates the bivariate function
        
        position: (x,y)
        center of the function: (xc,yc)
        control variables: Alpha and Beta will control the stringthof the vectors in x and y directions
        return: point in the bivariate function
    '''
    #k = 100000000 # parameter
    k = 10
    f = exp( -alpha*(position.x - center.x)/k**2 - beta*(position.y - center.y)/k**2 )
    #print(f)
    return f
 
def derivativeBivariate(alpha,beta,center,position):
    '''
        Calculates the bivariate function
        
        position: (x,y)
        center of the function: (xc,yc)
        control variables: Alpha and Beta will control the stringthof the vectors in x and y directions
        return: point in the bivariate function
    '''
    f = bivariateFunction(alpha,beta,center,position)
    dx = f * (-2*alpha*(position.x-center.x))
    dy = f * (-2*beta*(position.y-center.y))
    return vec(dx,dy)

def constrain_ang(ang,min,max):
    if ang > max:
        ang = max
    if ang < min:
        ang = min
    return ang

def random_color():
    """"
        Picks a random color R,G or B

        :return: color picked
        :rtype : tuple
    """
    a = random.uniform(0,255)
    rgbl=[a,0,0]
    random.shuffle(rgbl)
    return tuple(rgbl)

def limit(v2, max):
    """
        Limits magnitude of vector2

        :param v2: Vector2 to be normalized
        :type v2: pygame.Vector2
        :param max: maximum length of vector
        :type max: int
        :return v: returns vector 
        :rtype v: vector2
    """
    v = copy.deepcopy(v2)
    if v.length() > max:
        v.scale_to_length(max)
    return v

def limit3d(v3, max):
    """
        Limits magnitude of vector2

        :param v2: Vector2 to be normalized
        :type v2: pygame.Vector2
        :param max: maximum length of vector
        :type max: int
        :return v: returns vector 
        :rtype v: vector2
    """
    v = copy.deepcopy(v3)
    if v.length() > max:
        v /= v.length()
        v *= max
    return v
    
def constrain(v2,w,h):
    """
        Constrains movement of drone inside the canvas

        :param v2: Vector2 to be constrained
        :type v2: pygame.Vector2
        :param w: maximum width
        :type w: int
        :param h: maximum height
        :type h: int
        :return v2: returns vector within the limits
        :rtype v2: vector2
    """
    if v2.x > w:
        v2.x = w
    if v2.x < 0:
        v2.x = 0 
    if v2.y > h:
        v2.y = h
    if v2.y < 0:
        v2.y = 0
    return v2

def constrain3d(v3,w,h,alt):
    """
        Constrains movement of drone inside the canvas

        :param v2: Vector2 to be constrained
        :type v2: pygame.Vector2
        :param w: maximum width
        :type w: int
        :param h: maximum height
        :type h: int
        :return v2: returns vector within the limits
        :rtype v2: vector2
    """
    if v3.x > w:
        v3.x = w
    if v3.x < -w:
        v3.x = -w 
    if v3.y > alt:
        v3.y = alt
    if v3.y < 0.2:
        v3.y = 0.2
    if v3.z > h:
        v3.z = h
    if v3.z < -h:
        v3.z = -h
    return v3

class Aircraft(pg.sprite.Sprite):
    """
        Represents a simple visual animated drone 
        Can load sprites, rotate and update animation
    """
    def __init__(self):
        pg.sprite.Sprite.__init__(self)
        self.sprites = []

        for i in range(0,4):
            self.sprites.append(pg.image.load(f'models/Drone5/sprite_{i}.png').convert())
            

        self.atual = 0
        # inherited from the pygame sprite class it is the first element of the drone
        self.image = self.sprites[self.atual]
        # scales down drone sprites to (70,70)
        self.image = pg.transform.scale(self.image,(SIZE_DRONE*2,SIZE_DRONE*2))
        # rect is inherited from Sprite
        # defines the sprite's position on the screen
        # take the image size
        self.rect = self.image.get_rect()
        
        # pega o canto superior esquerdo, posição qualquer
        #self.rect.topleft = 100,100

    def colorize(self, newColor=(40,40,40)):
            """
            Create a "colorized" copy of a surface (replaces RGB values with the given color, preserving the per-pixel alphas of
            original).
            :param image: Surface to create a colorized copy of
            :param newColor: RGB color to use (original alpha values are preserved)
            :return: New colorized Surface instance
            """
            image = self.image.copy()

            # zero out RGB values
            #image.fill((0, 0, 0, 255), None, pg.BLEND_RGBA_MULT)
            # add in new RGB values
            image.fill(newColor[0:3] + (0,), None, pg.BLEND_RGBA_ADD)
            self.image = image

    def update(self, position, angle, size = SIZE_DRONE* PIX2M):
        
        # animation update speed is controle by this parameter
        self.atual += 1
        
        if self.atual >= len(self.sprites):
            self.atual = 0

        self.image = self.sprites[floor(self.atual)]
    
        # Rotates image -> angle should be in degrees
        # rotozoom(Surface, angle, scale) -> Surface
        self.image = pg.transform.rotozoom(self.image, -angle*180/pi - 90, .12)
        self.rect = self.image.get_rect()
        # positions center of rect in acual drone position
        self.rect.center = position.x,position.y

class FlowField():
    def __init__(self, resolution):

        self.cols =int(SCREEN_WIDTH/resolution)  # Columns of the grid
        self.rows = int(SCREEN_HEIGHT/resolution)  # Rows of the grid
        self.resolution = resolution # Resolution of grid relative to window width and height in pixels

        self.field = [[vec(random.uniform(0,1),random.uniform(0,1)) for col in range(self.cols)] for row in range(self.rows)] # create matrix 
        
    def draw(self, screen):

        blockSize = self.resolution #Set the size of the grid block
        #print(self.cols,self.rows)
        for x in range(0, SCREEN_WIDTH, blockSize):
            for y in range(0, SCREEN_HEIGHT, blockSize):
                rect = pg.Rect(x, y, blockSize, blockSize)
                pg.draw.rect(screen, (100,100,100), rect, 1)

class Npc_target(pg.sprite.Sprite):
    """
        Represents a simple visual animated tree 
        Can load sprites, rotate and update animation
    """
    def __init__(self):
        pg.sprite.Sprite.__init__(self)
        self.sprites = []

        #for i in range(1,4):
        self.sprites.append(pg.image.load(f'models/texture/wandering_trader1.png').convert())
        self.sprites[-1] =  pg.transform.rotozoom(self.sprites[-1], 0, 1)

        self.atual = 0
        # inherited from the pygame sprite class it is the first element of the drone
        self.image = self.sprites[self.atual]
        # scales down drone sprites to (70,70)
        #self.image = pg.transform.scale(self.image,(RADIUS_OBSTACLES,RADIUS_OBSTACLES))
        # rect is inherited from Sprite
        # defines the sprite's position on the screen
        # take the image size
        self.rect = self.image.get_rect()
        
        # pega o canto superior esquerdo, posição qualquer
        #self.rect.topleft = 100,100


    def update(self, position, angle, size = SIZE_DRONE* PIX2M):
        
        # animation update speed is controle by this parameter
        self.atual += .001
        if self.atual >= len(self.sprites)-1:
            self.atual = 0

        self.image = self.sprites[round(self.atual)]
    
        # Rotates image -> angle should be in degrees
        # rotozoom(Surface, angle, scale) -> Surface
        #self.image = pg.transform.rotozoom(self.image, 0, .2)
        self.rect = self.image.get_rect()
        # positions center of rect in acual drone position
        self.rect.midbottom = position.x,position.y+20

class Tree(pg.sprite.Sprite):
    """
        Represents a simple visual animated tree 
        Can load sprites, rotate and update animation
    """
    def __init__(self):
        pg.sprite.Sprite.__init__(self)
        self.sprites = []

        for i in range(1,2):
            self.sprites.append(pg.image.load(f'models/tree3/tree_{i}.png').convert())
            self.sprites[i-1] =  pg.transform.rotozoom(self.sprites[i-1], 0, .3)

        self.atual = 0
        # inherited from the pygame sprite class it is the first element of the drone
        self.image = self.sprites[self.atual]
        # scales down drone sprites to (70,70)
        #self.image = pg.transform.scale(self.image,(RADIUS_OBSTACLES,RADIUS_OBSTACLES))
        # rect is inherited from Sprite
        # defines the sprite's position on the screen
        # take the image size
        self.rect = self.image.get_rect()
        
        # pega o canto superior esquerdo, posição qualquer
        #self.rect.topleft = 100,100


    def update(self, position, angle, size = SIZE_DRONE* PIX2M):
        
        # animation update speed is controle by this parameter
        self.atual += .001
        if self.atual >= len(self.sprites)-1:
            self.atual = 0

        self.image = self.sprites[round(self.atual)]
    
        # Rotates image -> angle should be in degrees
        # rotozoom(Surface, angle, scale) -> Surface
        #self.image = pg.transform.rotozoom(self.image, 0, .2)
        self.rect = self.image.get_rect()
        # positions center of rect in acual drone position
        self.rect.midbottom = position.x,position.y+20
