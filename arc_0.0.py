import sys #Работа с системой
import os

import pygame
from pygame.locals import *
from pygame.color import *

import pymunk
from pymunk import Vec2d, BB
import pymunk.pygame_util
import pymunk.autogeometry

import random, time

brick_color=[THECOLORS['blue'],THECOLORS['blue'],THECOLORS['cyan'],THECOLORS['aquamarine'],THECOLORS['turquoise'],THECOLORS['steelblue'],THECOLORS['deepskyblue'],THECOLORS['midnightblue']]

w=600
he=600

global score
score=0
win_cond=0
lock_balls=0
level_count=0
cheat_win=False

win=False


collision_types={
	'ball':1,
	'brick':2,
	'bottom':3,
	'player':4
}

def spawn_ball(space,pos,direction):
	global lock_balls
	if lock_balls<=20:
		lock_balls+=1
	
		ball_body=pymunk.Body(1,pymunk.inf) #Указываем тип тела, передаём в наше тело свойства пространства(space)
		ball_body.position=pos
	
		ball_shape=pymunk.Circle(ball_body,5)
		ball_shape.color=random.choice(brick_color)
		ball_shape.elasticity = 1.0 #Сила отскока
		ball_shape.collision_type=collision_types['ball']
	
		ball_body.apply_impulse_at_local_point(Vec2d(direction))
	
		#Функция стабилизации скорости мяча
		def constant_velocity(body,gravity,damping,dt):
			body.velocity=body.velocity.normalized()*400
		ball_body.velocity_func=constant_velocity
	
		space.add(ball_body, ball_shape)

def setup_level(space,player_body):
	global lock_balls
	global win_cond
	win_cond=0
	lock_balls=0
	#Удаление динамических обьектов из пространства
	for s in space.shapes[:]:
		if s.body.body_type == pymunk.Body.DYNAMIC and s.body not in [player_body]:
			space.remove(s.body, s)

	spawn_ball(space,player_body.position+(0,40), random.choice([(1,10),(-1,10)]))

	#x_len=random.randrange(10,25)
	y_len=random.randrange(3,20)
	#win_cond=0

	for y in range(0,y_len):
		y = y * 10 + 360
		x_len=random.randrange(10,25)
		x_len_left=random.randrange(0,8)
		random_color=random.choice(brick_color)
		for x in range(x_len_left,x_len):
			x = x * 20 + 60

			brick_body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
			brick_body.position = x, y

			brick_shape = pymunk.Poly.create_box(brick_body, (20,10))
			brick_shape.elasticity = 1.0

			brick_shape.color = random_color
			brick_shape.group = 1

			brick_shape.collision_type = collision_types["brick"]
			space.add(brick_body, brick_shape)

			win_cond+=1
	global score
	global win_cond_2
	#score=0
	win_cond_2=win_cond//random.randrange(2,6) #Условие победы относительно кирпичеков
	def remove_brick(arbiter,space,data):
		global score
		score+=1
		global win_cond
		win_cond-=1
		brick_shape=arbiter.shapes[0]
		space.remove(brick_shape, brick_shape.body)
		print(win_cond)

	h = space.add_collision_handler(
		collision_types["brick"],
		collision_types["ball"])
	h.separate=remove_brick


def main():
	main_run=True
	while main_run:
		pygame.init()
		screen=pygame.display.set_mode((w,he))
		clock=pygame.time.Clock()
		running=True
		font=pygame.font.SysFont("Arial", 16)
		font_end=pygame.font.SysFont("Arial", 50)
	
		space=pymunk.Space()
		draw_options=pymunk.pygame_util.DrawOptions(screen)
	
		#Отрисовка, добавление полей
		static_lines=[pymunk.Segment(space.static_body, (50, 50), (50, 550), 2)
					,pymunk.Segment(space.static_body, (50, 550), (550, 550), 2)
					,pymunk.Segment(space.static_body, (550, 550), (550, 50), 2)
					]
		for line in static_lines:
			line.color=THECOLORS['lightgray']
			line.elasticity=1.0
		space.add(static_lines) #Добавление в space
	
		#Создание нижней границы и изменение её свойств
		bottom=pymunk.Segment(space.static_body,(50,50),(550,50),2)
		bottom.sensor=True
		bottom.collision_type=collision_types['bottom']
		bottom.color=THECOLORS['red']
		#Функция обработчика столкновений для нижней границы, удаляем всё что прикаснётся к ней
		def remove_first(arbiter,space,data):
			global score
			if score<0:
				score=0
			else:
				score-=5
	
			ball_shape=arbiter.shapes[0]
			space.remove(ball_shape,ball_shape.body)
			return True
		h=space.add_collision_handler(collision_types['ball'],collision_types['bottom'])
		h.begin=remove_first
		space.add(bottom)
	
		player_body=pymunk.Body(500,pymunk.inf)
		player_body.position=300,100
		player_shape=pymunk.Segment(player_body, (-50,0), (50,0), 8)
		player_shape.color=THECOLORS['red']
		player_shape.elasticity=1.0
		player_shape.collision_type=collision_types['player']
	
		def pre_solve(arbiter,space,data):
			set_=arbiter.contact_point_set
			if len(set_.points)>0:
				player_shape=arbiter.shapes[0]
				w_=(player_shape.b-player_shape.a).x
				delta=(player_shape.body.position-set_.points[0].point_a.x).x
				normal=Vec2d(0,1).rotated(delta/w_/2)
				set_.normal=normal
				set_.points[0].distance=0
			arbiter.contact_point_set=set_
			return True
	
		h = space.add_collision_handler( 
			collision_types["player"],
			collision_types["ball"])
		h.pre_solve = pre_solve
		#Ограничение движения игрока
		move_joint = pymunk.GrooveJoint(space.static_body, player_body, (100,100), (500,100), (0,0)) #Добавляем объект который будет ограничивать движение
		space.add(player_body, player_shape, move_joint)
		global state
		#Старт
		setup_level(space,player_body)
		#Главный цикл
		while running:
			global win_cond_2
			global win_cond
			global score
			global level_count
			global cheat_win

			for event in pygame.event.get():
				if event.type == QUIT: 
					running = False
					main_run=False
				elif event.type == KEYDOWN and (event.key in [K_ESCAPE, K_q]):
					running = False
					main_run=False
				elif event.type == KEYDOWN and event.key == K_p:
					pygame.image.save(screen, "breakout.png")
				elif event.type == KEYDOWN and event.key == K_LEFT:
					player_body.velocity = (-600,0)
				elif event.type == KEYUP and event.key == K_LEFT:
					player_body.velocity = 0,0
				    
				elif event.type == KEYDOWN and event.key == K_RIGHT:
					player_body.velocity = (600,0)
				elif event.type == KEYUP and event.key == K_RIGHT:
					player_body.velocity = 0,0
	
				elif event.type==KEYDOWN and event.key==K_n:
					#setup_level(space, player_body)
					win_cond_2=0
					#win_cond=0
					score=0
					level_count=0
					running =False
				elif event.type==KEYDOWN and event.key==K_w:
					cheat_win=True
	
				elif event.type==KEYDOWN and event.key==K_u:
					global lock_balls
					lock_balls=0

				if event.type == KEYDOWN and event.key == K_l:
					screen.blit(font.render(('Пауза'), 8, THECOLORS["white"]), (500,20))
					print('Пауза')
					while 1:
						event2=pygame.event.wait()
						if event2.type == KEYDOWN and event2.key == K_l:
							#screen.blit(font.render(('Пауза'), 8, THECOLORS["black"]), (500,20))
							break
		
				elif event.type == KEYDOWN and event.key == K_SPACE:
					spawn_ball(space,player_body.position+(0,40), random.choice([(1,10),(-1,10)]))

			screen.fill(THECOLORS["black"]) 
			
			space.debug_draw(draw_options)
			global win
			if win_cond==0 or cheat_win:
				print('if')
				if score>=win_cond_2 or cheat_win:
					screen.blit(font_end.render('ПОБЕДА!',50,THECOLORS['green']),(he//2-150,w//2-50))
					screen.blit(font.render(('press "R" to restart'), 8, THECOLORS["white"]), (he//2-150,w//2))
					screen.blit(font.render(('Пройдено уровней: '+str(level_count)), 8, THECOLORS["white"]), (he//2,w//2))
					keystate=pygame.key.get_pressed()
					if keystate[pygame.K_r]:
						win=True
						level_count+=1
						cheat_win=False
						running =False
				else:
					screen.blit(font_end.render('ПОРАЖЕНИЕ!',50,THECOLORS['red']),(he//2-150,w//2-50))
					screen.blit(font.render(str(round(score)), 8, THECOLORS["white"]), (he//2-150,w//2))
					keystate=pygame.key.get_pressed()
					if keystate[pygame.K_n]:
						win=False
	
			#FPS
			fps = 60
			dt = 1./fps
			space.step(dt)
			#Пишем текст
			screen.blit(font.render("FPS: " + str(clock.get_fps()), 1, THECOLORS["white"]), (0,0))
			screen.blit(font.render("Управление кнопками влево вправо , пробел для того чтобы создать новый мяч", 1, THECOLORS["darkgrey"]), (5,he - 35))
			screen.blit(font.render("N - рестарт , ESC или Q для выхода ", 1, THECOLORS["red"]), (5,he - 20))
	
			screen.blit(font.render(str(score)+' осталось: '+str(win_cond), 8, THECOLORS["white"]), (55,55))
			screen.blit(font.render(('Пройдено уровней: '+str(level_count)), 8, THECOLORS["white"]), (300,520))
			screen.blit(font.render(str(win_cond_2), 8, THECOLORS["white"]), (500,55))
	
			pygame.display.flip()
			clock.tick(fps)

if __name__ == '__main__':
    sys.exit(main())