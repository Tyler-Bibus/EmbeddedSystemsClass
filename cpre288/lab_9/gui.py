## @author Tyler Bibus
#General GUI on a ping/scan

import tkinter
from tkinter import *
from tkinter import ttk
import time

def start_scan():
    progress.start()
    
    for i in range(101):
        time.sleep(0.05)
        progress['value'] = i
        window.update_idletasks()
    progress.stop()


print("running")
window = tkinter.Tk()
msgVar = Label(window, text = 'CyBot Data Plot')
scanButton = Menubutton(window, text = "Scan")
scanButton.grid()
scanButton.menu = Menu(scanButton, tearoff = 0)
scanButton["menu"] = scanButton.menu
sVar = IntVar() #scan variable
aVar = IntVar() #about variable
scanButton.menu.add_checkbutton(label = 'Scan', variable = sVar)
scanButton.menu.add_checkbutton(label = 'About', variable = sVar)
scanButton.pack()
msgVar.pack()

#progress bar
progress = ttk.Progressbar(window, orient = "horizontal", length = 300, mode="determinate")
progress.pack(pady=20)

#start progress button
start_button = tkinter.Button(window, text="Start Scan", command=start_scan)
start_button.pack(pady=10)

window.mainloop()
