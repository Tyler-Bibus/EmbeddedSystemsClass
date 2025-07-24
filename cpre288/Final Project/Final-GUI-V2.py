# Author: Phillip Jones
# Date: 08/19/2022, updated 05/25/2023, updated 07/13/2023, updated 07/16/2023
# Description: Simple client starter code for collecting data from the cybot. Using the UART or 
#              Network socket interfaced to either a mock CyBot server, or the actual CyBot

# General Python tutorials (W3schools):  https://www.w3schools.com/python/

# Serial library:  https://pyserial.readthedocs.io/en/latest/shortintro.html 
#import serial
import time # Time library   
# Socket library:  https://realpython.com/python-sockets/  
# See: Background, Socket API Overview, and TCP Sockets  
import socket   

import os  # import function for finding absolute path  to this python script

import math

import tkinter as tk

# A little python magic to make it more convient for you to adjust where you want the data file to live
# Link for more info: https://towardsthecloud.com/get-relative-path-python 
absolute_path = os.path.dirname(__file__) # Absoult path  to this python script
relative_path = "./"   # Path  to sensor data file relative to this python script (./ means data file is in the same directory as this python script)
full_path = os.path.join(absolute_path, relative_path) # Full path  to sensor data file
filename = "SensorDataClean.txt" # Name of file you want to store sensor data from your sensor scan command

# Choose to create either a UART or TCP port socket to communicate with Cybot (Not both!!)
# UART BEGIN
#cybot = serial.Serial("COM100", 115200)  # UART (Make sure you are using the correct COM port and Baud rate!!)
# UART END

# TCP Socket BEGIN (See Echo Client example): https://realpython.com/python-sockets/#echo-client-and-server
HOST = "192.168.1.1"  # The server"s hostname or IP address
PORT = 288        # The port used by the server
cybot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create a socket object                    
cybot = cybot_socket.makefile("rbw", buffering=0)  # makefile creates a file object out of a socket:  https://pythontic.com/modules/socket/makefile
CYBOT_RADIUS = .19
SERVO_RADIUS = .14
CM = 100.0

objects = []
nodes = []
#intersections = []
# TCP Socket END

# Send some text: Either 1) Choose "Hello" or 2) have the user enter text to send
#send_message = "Hello\n"                            # 1) Hard code message to "Hello", or

class cybot_status():
        def __init__(self) -> None:
                self.x:float=0
                self.y:float=0
                self.angle:float=90
                self.is_connected:bool = False
                self.cy_path:list[aStar_node] = []
                self.cy_target:tuple[float,float] = (0,0)
                self.final_target = None
        def get_rad(self) -> float:
                return self.angle*math.pi/180
        def set_rad(self, radians:float) -> None:
                self.angle = radians * 180 / math.pi
        def get_map(self) -> tuple:
                return toMap(self.x,self.y)
        def set_target(self,target:tuple,p_nodes:list):
                self.cy_target = target
                min_cy_dist = 999
                min_goal_dist = 999
                min_cy_node = p_nodes[0]
                min_goal_node = p_nodes[0]
                #find closest node to cybot and goal
                for i in nodes:
                        new_cy_dist = i.dist_to(self.x,self.y)
                        if new_cy_dist<min_cy_dist:
                                min_cy_dist = new_cy_dist
                                min_cy_node=i

                        new_goal_dist = i.dist_to(self.cy_target[0],self.cy_target[1])
                        if new_goal_dist<min_goal_dist:
                                min_goal_dist = new_goal_dist
                                min_goal_node=i

                #print("min cy dist",min_cy_dist)
                #print("min goal dist",min_goal_dist)
                self.cy_path = aStar(min_cy_node,min_goal_node)
                term_print("path to "+str(target)+" through "+str(len(self.cy_path))+" nodes\n")
        def move_to_target(self,x,y)->int:#returns 0 on sucess, 0> on error
                error = 0
                dy = y-self.y
                dx = x-self.x
                angle = math.atan2(dy, dx) - math.atan2(0, 1)#in radians
                da = angle-self.get_rad()
                dist=math.sqrt(dy**2+dx**2)#in meters
                #term_print(str(angle/math.pi)+"r "+str(dist)+"m\n")
                truAngle = -int(math.degrees(da))%360
                if abs(truAngle)> 180:
                        truAngle = truAngle - 360
                command = "t "+str(truAngle) + "\n"
                print("command:"+command)
                response = self.call(command)
                term_print("turn : "+response)
                self.set_rad(self.get_rad() + da)
                update_graphics()
                ##TODO Confrim that its /100 not * 100 for cm because the command is in cm
                command = "m "+str(dist*100)  + "\n"
                print("command: ",command)
                response = self.call(command)
                term_print("response:"+ response)
                match response[0]:
                        case "d":
                                pass
                        case "b":
                                objects.append(object(90,CYBOT_RADIUS,.1,self))
                                dist -= float(response.split()[1])/100#to meteter
                                print("response:",response,"dist:",dist)
                                error = 1
                        case "h":
                                objects.append(object(90,CYBOT_RADIUS,.1,self, True))
                                dist -= float(response.split()[1])/100#to meters
                                error = 2
                        case "w":
                                objects.append(object(90,CYBOT_RADIUS,.1,self, is_bound=True))
                                dist -= float(response.split()[1])/100#to meters
                                error = "boundary"
                #cybot.write(command.encode)
                delta=0
                if(error!=0):
                        delta = CYBOT_RADIUS
                        move(-delta*100)
                self.x+=(dist-delta)*math.cos(self.get_rad())
                self.y+=(dist-delta)*math.sin(self.get_rad())

                update_graphics()
                return error
        def call(self,command:str)->str:
                #for c in command:
                        #print("wrote",cybot.write(c.encode()),"bytes")
                #print("wrote",cybot.write(command.encode()),"bytes")
                cybot.write(command.encode())
                rx_message = bytearray(1)
                response = ""
                rx_message = cybot.readline()
                while(rx_message.decode()!="d\n"):
                        response+=rx_message.decode()
                        rx_message = cybot.readline()
                response+=rx_message.decode()
                return response

        def draw(self,canvas)->None:
                cybot_map_coords = toMap(self.x,self.y)
                theta = Status.get_rad()
                cybot_point_coords = toMap(math.cos(theta)*CYBOT_RADIUS+self.x,math.sin(theta)*CYBOT_RADIUS+self.y)
                create_circle(cybot_map_coords[0],cybot_map_coords[1],CYBOT_RADIUS*100,map_canvas, "red")
                map_canvas.create_line(cybot_map_coords[0],cybot_map_coords[1],cybot_point_coords[0],cybot_point_coords[1],fill="black",width=3)
                if self.final_target!=None:
                        final_point_coords = toMap(self.final_target[0],self.final_target[1],)
                        create_circle(final_point_coords[0],final_point_coords[1],10,map_canvas, "lime")

                

class object():
        def __init__(self,angle_too:int, distance:float,linear_width:float,status:cybot_status,is_hole:bool=False,is_bound=False) -> None:
                self.distance = distance
                self.rad = linear_width/2
                effective_angle = status.angle+angle_too-90
                #print("effective angle : "+str(effective_angle))
                self.x= status.x + SERVO_RADIUS*math.cos(status.get_rad()) + math.cos(math.radians(effective_angle))*distance
                self.y= status.y + SERVO_RADIUS*math.cos(status.get_rad()) + math.sin(math.radians(effective_angle))*distance
                self.is_hole=is_hole
                self.is_bound = is_bound
                #cull nodes
                min_rad = 0
                if(self.is_hole or self.is_bound):
                        min_rad += self.rad
                else:
                        min_rad += self.rad + CYBOT_RADIUS
                toCull = []
                for n in nodes:
                        dist_too = n.dist_to(self.x,self.y)
                        if dist_too>min_rad*2:
                                continue
                        if dist_too<=min_rad:
                                #print(n)
                                #DONE:reconnect valid nodes
                                for c in n.near:
                                        for ci in n.near:
                                                if ci not in n.near:
                                                        continue
                                                if ci == c:
                                                        continue
                                                if ci in c.near:
                                                        continue
                                                if not c.line_interrupts(ci,min_rad,self.x,self.y):
                                                        c.near.append(ci)
                                                        ci.near.append(c)
                                toCull.append(n)
                                pass
                                
                        else:
                                #cull connections:
                                for n in nodes:
                                        for c in n.near:
                                                if(n.line_interrupts(c,min_rad,self.x,self.y)):
                                                        #print("culled connection between",n,c)
                                                        n.near.remove(c)
                                                        c.near.remove(n)
                for n in toCull:
                        n.cull()
                
                        
        def __str__(self) -> str:
                return str(self.rad)+" at "+str(self.x)+","+str(self.y)
        def angle_to(self, o:object):
                dy =  o.y-self.y
                dx = o.x-self.x
                angle = math.atan2(dy, dx) - math.atan2(0, 1)#in radians
                return math.degrees(angle)
        def dist_to(self,o:object):
                dy =  o.y-self.y
                dx = o.x-self.x
                dist = math.sqrt(dy**2+dx**2)
                return dist
        def draw(self,canvas):
                mapcoord = toMap(self.x,self.y)
                if not (self.is_hole or self.is_bound):
                        #draw warning zone
                        create_circle(mapcoord[0],mapcoord[1],(self.rad+CYBOT_RADIUS)*100,canvas,"orange")
                #draw object
                if(self.is_bound):
                        return create_circle(mapcoord[0],mapcoord[1],self.rad*100,canvas, "white")
                return create_circle(mapcoord[0],mapcoord[1],self.rad*100,canvas, "black")
        
class aStar_node():
        def __init__(self,x,y):
                self.x=x
                self.y=y
                self.near = []#all adjacent nodes
                self.visited = False
        def __str__(self) -> str:
                return str(self.x)+","+str(self.y) + " with " + str(len(self.near)) + " near "
        def draw(self,canvas):
                mapcoord = toMap(self.x,self.y)
                for n in self.near:
                        nearcoord = toMap(n.x,n.y)
                        canvas.create_line(mapcoord[0],mapcoord[1],nearcoord[0],nearcoord[1],fill="blue",width=1)
                if self.visited:
                     return create_circle(mapcoord[0],mapcoord[1],3,canvas, "black")   
                return create_circle(mapcoord[0],mapcoord[1],3,canvas, "green")
        def cost(self, node):
                #distance to node
                return math.sqrt( (self.x-node.x)**2 + (self.y-node.y)**2 )
        def dist_to(self,x,y):
                return math.sqrt( (self.x-x)**2 + (self.y-y)**2 )
        def line_interrupts(self,node,radius,ob_x,ob_y):#ret true if line between self and node passes through circule of radius and xy
                #compute line between self and node
                if node.x-self.x==0:#slope is infinite/ straight vertical
                        #if in bounds
                        if(ob_y>min(self.y,node.y) and ob_y<max(self.y,node.y)):
                                return True
                        return False
                m = float(node.y-self.y) / (node.x-self.x)
                #(y-self.y)=m(x-self.x)
                if(m==0):#slope is 0 / straight horizontal
                        #if in bounds
                        if(ob_x>min(self.x,node.x) and ob_x<max(self.x,node.x)):
                                if(abs(self.y-ob_y)<radius):
                                        return True
                        return False
                #compute line between prev line and circle
                m_inv = -1.0/m
                #(y-ob_y)=m_inv()(x-ob_x)

                #compute intersection point
                x_insersection=(ob_x*m_inv-ob_y-self.x*m+self.y)/(m_inv-m)
                y_intersection=m*(x_insersection-self.x)+self.y
                
                

                #check if intersection less than radius
                if(x_insersection>min(self.x,node.x) and x_insersection<max(self.x,node.x) and y_intersection>min(self.y,node.y) and y_intersection<max(self.y,node.y)):
                        #intersections.append((x_insersection,y_intersection))
                        if(math.sqrt((x_insersection-ob_x)**2+(y_intersection-ob_y)**2))<radius:
                #check if intersection within bounds
                                return True
                return False
        def cull(self):
                #remove self from all near
                for node in self.near:
                        node.near.remove(self)
                nodes.remove(self)

def aStar(start:aStar_node, goal:aStar_node):#DONE:finsih implementation
        #print("astar:",start,goal)
        frontier = [start]
        came_from = dict()
        cost_so_far = dict()
        came_from[start] = None
        cost_so_far[start] = 0

        #print(frontier)
        found_goal = False

        while frontier:
                current:aStar_node = frontier.pop(0)

                if current == goal:
                        found_goal=True
                        break
                
                for next in current.near:
                        new_cost:int = cost_so_far[current] + current.cost(next)
                        if next not in cost_so_far or new_cost < cost_so_far[next]:
                                cost_so_far[next] = new_cost
                                priority = new_cost + goal.cost(next)
                                #push to frontier
                                if not frontier:
                                        frontier = [next]
                                elif priority>=cost_so_far[frontier[-1]]:
                                        frontier.append(next)
                                else:
                                        for i in range(len(frontier)):
                                                if cost_so_far[frontier[i]]>=priority:
                                                        frontier.insert(i,next)
                                                        break
                                came_from[next] = current

        ##build path
        if not found_goal:
                return []
        result = []
        current = goal
        while(current!=start):
                result.append(current)
                current=came_from[current]
        result.reverse()
        return result

def create_circle(x, y, r, canvas,color): #center coordinates, radius
    x0 = x - r
    y0 = y - r
    x1 = x + r
    y1 = y + r
    return canvas.create_oval(x0, y0, x1, y1, fill=color)

def toMap(x=0,y=0)->tuple:
        return (MAP_BOUND/2+x*100,MAP_BOUND/2-y*100)
def fromMap(x=0,y=0)->tuple:
        return ((x-MAP_BOUND/2)/100,(MAP_BOUND/2-y)/100)

def connect():
        try:
                global cybot_socket
                global cybot_is_connected
                global cybot
                cybot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create a socket object
                cybot_socket.connect((HOST, PORT))   # Connect to the socket  (Note: Server must first be running)
                cybot_is_connected = True
                for b in comm_buttons:
                        b["state"] = "active"
                button_c["state"]="disabled"
                button_dc["state"]="active"
                cybot = cybot_socket.makefile("rbw", buffering=0)  # makefile creates a file object out of a socket:  https://pythontic.com/modules/socket/makefile
                term_print("cybot connected\n")

        except TimeoutError:
                 term_print("connecton timeout\n")
#DONE:make compliant with new command networking
def move(dist_cm):
        command = "m "+str(int(dist_cm))+"\n"
        print(command)
        response = Status.call(command)
        term_print("moving fward: "+response)
def turn(angle_deg):
        command = "t "+str(int(angle_deg))+"\n"
        response = Status.call(command)
        term_print("turning ccw: "+response)
        Status.angle-=int(angle_deg)
def scan_cmd():
        #DONE: make complinat with new code, add detected objects
        command = "s\n"
        term_print("Requested Sensor scan from Cybot:\n")
        cybot.write(command.encode())
        rx_message = bytearray(1) # Initialize a byte array
        rx_message = cybot.readline()   # Wait for sensor response, readline expects message to end with "\n"
        while (rx_message.decode() != "d\n"): # Collect sensor data until "END" recieved
            print("rec:",rx_message.decode())
            #print("in while loop\n")
            raw_obj = rx_message.decode().split()
            o = object(float(raw_obj[0]),float(raw_obj[1])/100,float(raw_obj[2])/100,Status)
            objects.append(o)
            rx_message = cybot.readline()   # Wait for sensor response, readline expects message to end with "\n"
        #exec(open("./CyBot-Plot-Sensor-Scan-Values-From-File.py").read())
        #detect_square()
        term_print("scan finished")

def dsiconect_cmd():
        #command = "q\n"
        #cybot.write(command.encode())
        term_print("Client exiting, and closing file descriptor, and/or network socket\n")
        time.sleep(2) # Sleep for 2 seconds
        cybot.close() # Close file object associated with the socket or UART
        cybot_socket.close()  # Close the socket (NOTE: comment out if using UART interface, only use for network socket option)
        for b in comm_buttons:
                b["state"] = "disabled"
        button_dc["state"]="disabled"
        button_c["state"]="active"
        
def do_path_cmd():
        if Status.cy_path:
                while Status.cy_path:
                        target_node = Status.cy_path.pop(0)
                        result = Status.move_to_target(target_node.x,target_node.y)
                        if result!=0:#nonzero response, abort
                                term_print("aborted path:error code "+str(result))
                                Status.set_target(Status.cy_target,nodes)
                                return
                        target_node.visited=True
                result = Status.move_to_target(Status.cy_target[0],Status.cy_target[1])
                if result:#if error
                        Status.set_target(Status.cy_target,nodes)
                        return result#abort

def detect_square():
        DIST_ERROR = .05#meters
        for o0 in objects:
                if o0.is_hole:
                        continue
                same = {}
                r2 = {}
                for o1 in objects:
                        if o1.is_hole or o0 == o1:
                                continue

                        dist_0_1 = o0.dist_to(o1)
                        #check if dist is a same or r2 value
                        for o2 in objects:
                                if o1.is_hole or o2 == o1 or o2==o0:
                                        continue
                                dist_0_2 = o0.dist_to(o2)
                                if dist_0_1>dist_0_2 - DIST_ERROR and dist_0_1<dist_0_2 + DIST_ERROR:
                                        same[dist_0_1] = (o1,o2)
                                        continue
                                
                                if dist_0_1*math.sqrt(2)> (dist_0_2-DIST_ERROR) and dist_0_1*math.sqrt(2)> (dist_0_2-DIST_ERROR):
                                        r2[dist_0_1] = (o1,o2)#o2 is sqrt(2)*o1 length
                                        continue
                        
                #check if r2 and same contain same key(side length)
                for key_same in same:
                        for key_r2 in r2:
                                if (r2[key_r2][0]) in same[key_same] and (key_r2>key_same-DIST_ERROR and key_r2<key_same+DIST_ERROR):
                                        #square found
                                        print("square found"+str(r2[key_r2]),str(same[key_same]),str(key_r2))
                                        print("square:",o0,same[key_same][0],same[key_same][1],r2[key_r2][1])
                                        #calculate center of square
                                        center_x = (r2[key_r2][1].x + o0.x)/2
                                        center_y = (r2[key_r2][1].x + o0.x)/2
                                        Status.final_target = (center_x,center_y)
                                        update_graphics()
                                        return
                                

def clear_all_inside_frame(frame):
    # Iterate through every widget inside the frame
    for widget in frame.winfo_children():
        widget.destroy()  # deleting widget

def update_graphics():
        #draw oject map
        map_canvas.delete("all")
        #intersections = []
        for o in objects:
                #print(o)
                #print(MAP_BOUND/2 + o.x*100,MAP_BOUND/2 - o.y*100)
                o.draw(map_canvas)
        for n in nodes:
                n.draw(map_canvas)
        
        #draw cybot
        Status.draw(map_canvas)

        #draw current path
        target_coord = toMap(Status.cy_target[0],Status.cy_target[1])
        create_circle(target_coord[0],target_coord[1],10,map_canvas,color="green")
        if(Status.cy_path):
                Tuu = toMap(Status.cy_path[0].x,Status.cy_path[0].y)
                map_canvas.create_line(Status.get_map()[0],Status.get_map()[1],Tuu[0],Tuu[1],fill="green",width=2)
                for n in range(1,len(Status.cy_path)):
                        Frum = toMap(Status.cy_path[n-1].x,Status.cy_path[n-1].y)
                        Tuu = toMap(Status.cy_path[n].x,Status.cy_path[n].y)
                        create_circle(Tuu[0],Tuu[1],5,map_canvas,color="green")
                        map_canvas.create_line(Frum[0],Frum[1],Tuu[0],Tuu[1],fill="green",width=2)

        #debug
        #for i in intersections:
                #map_coords = toMap(i[0],i[1])
                #create_circle(map_coords[0],map_coords[1],3,map_canvas,"red")

        window.update_idletasks()
        window.update()
        time.sleep(.1)

def term_print(inpStr):
        term_text.insert("end", inpStr+'>')
        term_text.see("end")
        update_graphics()

#TODO TEST
def flush_cmd():
        command = "f\n"
        term_print("Requested a flush:\n")
        cybot.write(command.encode())
        rx_message = bytearray(1) # Initialize a byte array
        rx_message = cybot.readline()
        while(rx_message.decode() != "f\n"):
                rx_message = cybot.readline()
        print("Flush finished")
        rx_message = cybot.readline()
        print("Recieved:", rx_message.decode())

#TODO TEST
def song_cmd():
        command = "o\n"
        term_print("Singing Rango:\n")
        cybot.write(command.encode())
        rx_message = bytearray(1) # Initialize a byte array
        rx_message = cybot.readline() # Wait until done recieved
        print("Song Recieved:", rx_message.decode())

#UNFINISHED, too difficut, will do manual drive
def auto_cmd():
    #do a spiral search, no need to keep track of frontier
    #implement a while loop that uses Astar to path to the next "corner", slowly increasing the 
    current_position = (0,0)
    directions = [(-1,0), (0,1),(1,0),(0,-1)] #up right down and left
    visited = set()
    visited.add(current_position)
    
    layer = 1
    while(Status.final_target==None):
        print("cur_pos:"+str(current_position))
        #spiral search
        for dx, dy in directions * layer:


            #check if we found target: target TBD but im assuming itll be 4 skinny pillars in a square
            #explanation: a square has 2 nodes of the same distance from it and one node sqrt(2) that distance
            #from it. if a object has these 3 other objects, it is a square
            detect_square()


                        

            next_target = (current_position[0] + dx, current_position[1] + dy)
            
            #check for boundaries, TODO create way to add boundaries via hole detection Otherwise set default maximum bounds (assume on edge of each side)
            
            #check if we already visited
            if next_target in visited:
                continue
            
            #check if obstical
            
            
            #move to target
            Status.set_target(next_target,nodes)
            result = do_path_cmd()
            #if success, mark as visited
            if result == 0:
                visited.add(next_target)
            
            else:
                #else retry, then mark as unpathable
                pass
            scan_cmd()
                

            ##DELETE BREAKS UPON COMPLETION, AVOIDS INFINITE LOOP
            #break
        #break
        Status.set_target(Status.final_target,nodes)
        result = do_path_cmd()

        

Status = cybot_status()

window = tk.Tk()
MAP_BOUND = 600
map_frame = tk.Frame(window, width=MAP_BOUND, height=MAP_BOUND,highlightbackground="black",highlightthickness=1)
map_frame.grid_propagate(0)
map_frame.pack_propagate(False)
map_frame.grid(row=0, column=0, padx=5, pady=5,sticky="nsew")

connect_frame = tk.Frame(window, width=MAP_BOUND/2, height=MAP_BOUND/2,highlightbackground="black",highlightthickness=1)
connect_frame.grid(row=0, column=2, padx=5, pady=5,sticky="nsew")

button_frame = tk.Frame(window, width=MAP_BOUND/2, height=MAP_BOUND,highlightbackground="black",highlightthickness=1)
button_frame.grid(row=0, column=1, padx=5, pady=5,sticky="nsew")

terminal_frame = tk.Frame(window, width=0, height=0,highlightbackground="black",highlightthickness=1)
#terminal_frame.grid_propagate(0)
#terminal_frame.pack_propagate(False)
terminal_frame.grid(row=0, column=3, padx=5, pady=5,sticky="nsew")


dist_text = tk.Text(
        button_frame,
        bg="black",
        fg="white",
        width=5,
        height=1,
        wrap="none",
)
button_m = tk.Button(
    button_frame,
    text="move",
    width=5,
    #height=5,
    bg="black",
    fg="white",
    command = lambda : move(dist_text.get("1.0","end")),

)
angle_text = tk.Text(
        button_frame,
        bg="black",
        fg="white",
        width=5,
        height=1,
        wrap="none",
)
button_t = tk.Button(
    button_frame,
    text="turn",
    width=5,
    #height=5,
    bg="black",
    fg="white",
    command = lambda: turn(angle_text.get("1.0","end")),
)
button_s = tk.Button(
    button_frame,
    text="Scan",
    width=5,
    #height=5,
    bg="black",
    fg="white",
    command = scan_cmd,
)
button_l = tk.Button(
    button_frame,
    text="auto",
    width=5,
    #height=5,
    bg="black",
    fg="white",
    command = auto_cmd,
)
button_path = tk.Button(
    button_frame,
    text="path",
    width=5,
    #height=5,
    bg="black",
    fg="white",
    command = do_path_cmd,
)

button_flush = tk.Button(
    button_frame,
    text="flush",
    width=5,
    bg="black",
    fg="white",
    command=flush_cmd,
)

#plays song
button_o = tk.Button(
    button_frame,
    text="song",
    width=5,
    bg="black",
    fg="white",
    command=song_cmd,
)

button_c = tk.Button(
    connect_frame,
    text="connect",
    width=5,
    #height=5,
    bg="black",
    fg="white",
    command = connect,
)
button_dc = tk.Button(
    connect_frame,
    text="dc",
    width=5,
    #height=5,
    bg="black",
    fg="white",
    command = dsiconect_cmd,
)

terminal_frame.grid_propagate(0)
term_text = tk.Text(
        terminal_frame,
        bg="black",
        fg="white",
        height=20,
        width=50,
)
terminal_frame.grid_propagate(0)



map_canvas = tk.Canvas(
        map_frame,
        bg="grey",
        width=MAP_BOUND,
        height=MAP_BOUND,
)

map_canvas.pack()

dist_text.pack()
button_m.pack()
angle_text.pack()
button_t.pack()
button_s.pack()
button_m.pack()
button_l.pack()
button_path.pack()
button_flush.pack()
button_o.pack()

button_c.pack()
button_dc.pack()

terminal_frame.grid_propagate(0)

term_text.pack()

terminal_frame.grid_propagate(0)

map_canvas.pack()

def getorigin(eventorigin):
    x0 = eventorigin.x
    y0 = eventorigin.y
    target = fromMap(x0,y0)
    #print("destination set to "+str(target)+"\n")
    Status.set_target(target,nodes)

def getKeyPress(event):
        if(event.keysym=="Return"):
                line = term_text.get("end-2c linestart", "end-2c lineend")
                line = line.replace('>','')
                #print(line)
                term_print("")
                response = Status.call(line+'\n')
                term_print(response)

#mouseclick event
map_canvas.bind("<Button 1>",getorigin)
term_text.bind("<KeyRelease>", getKeyPress)

term_print("welcome to the gui\n")

comm_buttons = [button_m,button_t,button_m]#button_l,button_path]

for b in comm_buttons:
        b["state"] = "disabled"
button_dc["state"]="disabled"


#place nodes in hex pattern
DENSITY = 24.0
PADDING = 0
NODE_RANGE:int = int((MAP_BOUND-PADDING)/DENSITY)-1
#print("range: "+str(NODE_RANGE))
for y in range(-NODE_RANGE+1,NODE_RANGE):
        for x in range(-NODE_RANGE+1,NODE_RANGE):
                #print("x, y : "+str((x,y)))
                nodes.append(aStar_node(x*DENSITY/100 + (y%2)*(DENSITY/2/100), y*DENSITY/100))

#connect nodes
num_in_row = 2*(NODE_RANGE)-1
#print("num_in_row:" + str(num_in_row))
for n in range(len(nodes)):
        #print(n)
        parity:int = int(int(n)/int(num_in_row)+1 )%2
        for c in ((1,  0), ( parity, -1), (parity-1, -1),(-1,  0), (parity-1, 1), ( parity, 1)):
                #print(c)
                n_coord = (n%num_in_row,int(n/num_in_row))
                near_coord = (int(c[0]+n_coord[0]),int(c[1]+n_coord[1]))
                #check if target node out of bounds
                if(near_coord[0]>=num_in_row or near_coord[0]<0 or near_coord[1]>=num_in_row or near_coord[1]<0):
                        #print("out of bounds")
                        continue
                #print("c:"+str(c)+"+parity:"+str(parity)+"->"+str(near_coord))
                near_idx = int(near_coord[0]+near_coord[1]*num_in_row)
                #print("near_idx:"+str(near_idx))
                if(near_idx>=0 and near_idx<len(nodes)):
                        #print("added node "+str(nodes[near_idx])+ " to "+str(n))
                        nodes[n].near.append(nodes[near_idx])

#for n in nodes:
        #print(n)

objects = []#[object(90,1,.07,Status),object(-20,.5,.09,Status),object(30,1.2,.09,Status)]
#objects = [object(45,1,.1,Status),object(135,1,.1,Status),object(25,2,.1,Status),object(315,1,.1,Status)]

Status.set_target((1.5,1.5),nodes)

detect_square()

while(1):
        update_graphics()