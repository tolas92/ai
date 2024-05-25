import pygame
import spritesheet

pygame.init()

SCREEN_WIDTH = 1000
SCREEN_HEIGHT = 1000

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption('Spritesheets')

sprite_sheet_image = pygame.image.load('icon_71_frames.png').convert_alpha()
sprite_sheet = spritesheet.SpriteSheet(sprite_sheet_image)

BG = (50, 50, 50)
BLACK = (0, 0, 0)

#create animation list
animation_list=[]
animation_steps=[7,9,16,9,19,8]
action=5
last_update=pygame.time.get_ticks()
animation_cooldown= 250
frame=0
step_counter=0

for animation in animation_steps:
	temp_img_list=[]
	for _ in range(animation):

		temp_img_list.append(sprite_sheet.get_image(step_counter,900,900,1,BLACK))
		step_counter +=1
	animation_list.append(temp_img_list)

run = True
while run:

	#update background
	screen.fill(BG)

	#update animation
	current_time=pygame.time.get_ticks()
	if current_time-last_update>=animation_cooldown:
		frame+=1
		last_update=current_time
		if frame>=len(animation_list[action]):
			frame=0
    
	

	#show frame image
	screen.blit(animation_list[action][frame], (0, 0))

	#event handler
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			run = False

	pygame.display.update()

pygame.quit()