import tkinter as tk
from funciones import moveToAngle, moveToPos
pos =[1,1,1]
global angles
global data
    
# Create a Tkinter window
window = tk.Tk()
# Create a Tkinter canvas
canvas = tk.Canvas(window, width=600, height=400, bg='green')
# Draw the cartesian axis (or grid) on the canvas
canvas.create_line(0, 200, 600, 200, width=1)  # x-axis
canvas.create_line(300, 0, 300, 600, width=1)  # y-axis
## elementos dentro de la ventana



# Bind a mouse event to the canvas to draw with the mouse

def draw(event):# este evento que sucede por el mouse - despues en automatico
    pos = []
    # Get the current mouse position
    x, y, z = event.x, event.y, Slider1.get()
    pos = x,y,z
    z=z*0.01
    # Draw a dot on the canvas at the current mouse position 
    canvas.create_oval(x-z, y-z, x+z, y+z, fill='black')
    text.insert('1.0', 'x: '+ str(pos[0]) + 'y: ' + str(pos[1])+ 'z: ' + str(pos[2])+ '\n')
    #text.insert('1.0', 'lidar: '+ str(data[0]) + 'gyrohombro: ' + str(data[1])+ 'gyrocodo: ' + str(data[2])+ '\n')

def moveYprint():
    moveToAngle(moveToPos(pos[0], pos[1], pos[2]))
    text.insert('1.0', 'Angles '+ str(angles))


text = tk.Text(window, height=8)
text.pack(side=tk.BOTTOM)
text.insert('1.0', 'coordenadas')
move_button = tk.Button(window, text='move',  command= lambda: moveYprint()) ##NO REACCIONA - aca movetoangle(movetopos)
move_button.pack( ipadx=4, ipady=4, expand=True, side=tk.BOTTOM)
stop_button = tk.Button(window, text='stop',  command="")
stop_button.pack( ipadx=4, ipady=4, expand=True, side=tk.BOTTOM)
sleep_button = tk.Button(window, text='sleep',  command="")
sleep_button.pack( ipadx=4, ipady=4, expand=True, side=tk.BOTTOM)
Slider1 = tk.Scale(window, from_=600, to=0, orient=tk.VERTICAL, length=400)
Slider1.pack(side=tk.RIGHT)