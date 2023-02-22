# import pygame module in this program
import pygame
 
# activate the pygame library
# initiate pygame and give permission
# to use pygame's functionality.
pygame.init()
 
 
# create the display surface object
# (x, y) is the height and width of pygame window
win=pygame.display.set_mode((400, 400))
 
# set the pygame window name
pygame.display.set_caption("Train Info")
 
# setting the pygame font style(1st parameter)
# and size of font(2nd parameter)
Font1=pygame.font.SysFont('freesansbold.ttf',  60)
Font2=pygame.font.SysFont('freesansbold.ttf',  40)
 
# define the RGB value for white,
# green, yellow, orange colour
white=(255, 255, 255)
yellow=(255, 255, 0)
green=(0, 255, 255)
orange=(255, 100, 0)
black=(0, 0, 0)
a = 100
done=False
 
# Split the text into letters
# 3rd parameter is font colour and
# 4th parameter is Font background
Speed=Font1.render("Speed: "+ str(a) +"MPH", True, black)
Control=Font1.render("Control: "+ str(a), False, black)
Distance=Font2.render("Odometer: "+ str(a)+"Miles", False, black)
Station=Font2.render("Next Station: "+ str(a)+"Miles", False, black)
Crossing=Font2.render("NA", False, black)
 
# assigning values to
# i and c variable
i=0
c=1
 
# infinite loop
while not done:
    if(i>=820):
        i=0
        c+=1
        pygame.time.wait(500)
         
    # completely fill the surface object
    # with white color
    win.fill(white)
 
        # Scrolling the text in diagonal
        # on right side of the Screen.
        # copying the text surface object
        # to the display surface object 
        # at the center coordinate.
    Speed=Font1.render("Speed: "+ str(i) +"MPH", True, white)
    Control=Font1.render("Control: "+ str(i), False, white)
    Distance=Font2.render("Odometer: "+ str(i)+"Miles", False, white)
    Station=Font2.render("Next Station: "+ str(i)+"Miles", False, white)
    win.blit(Speed, (10, 10))
    win.blit(Control, (10, 70))
    win.blit(Distance, (10, 140))
    win.blit(Station, (10, 190))
    i+=80

    # Draws the surface object to the screen.
    pygame.display.update()
     
    # iterate over the list of Event objects
    # that was returned by pygame.event.get() method
    for event in pygame.event.get():
        if(event.type==pygame.QUIT):
            done=True
    #Delay with 5ms
    pygame.time.wait(500)
pygame.quit()