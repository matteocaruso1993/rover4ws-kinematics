import tkinter as tk


class RoverPlotterApp(tk.Tk):
    def __init__(self, parent) -> None:
        super().__init__(parent)
        self.parent = parent
        self.reset()
        

    def reset(self):
        self.geometry('1600x800')
        self.grid()
        button = tk.Button(self,anchor="center",bg="green")
        button.pack(
        ipadx=5,
        ipady=5,
        expand=True
)

class RoverBaseApp(tk.Tk):
    def __init__(self, parent) -> None:
        super().__init__(parent)
        self.parent = parent
        self.reset()

    def reset(self):
        self.l = tk.Label(text="Four Wheel Stering Rover ICR App v1.0.1", font=50)
        self.l.grid(row=0,column=5)
        self.but1 = tk.IntVar()  
        self.but2 = tk.IntVar()  
        self.but3 = tk.IntVar()

        self.but1.set(1)
        self.but2.set(0)
        self.but3.set(0)

        c1 = tk.Checkbutton(
            self, text="Keyboard Control",
            variable=self.but1,
            command=self._check1)

        c2 = tk.Checkbutton(
            self, text="Mouse Control",
            variable=self.but2,
            command=self._check1)


        c3 = tk.Checkbutton(
            self, text="Keyboard Control",
            variable=self.but2,
            command=self._check1)


        c1.grid(column=0,row=10)
        c2.grid(column=0,row=20)
        c3.grid(column=0,row=30)




    def _check1(self):
        self.but2 = 0
        self.but3 = 0
        

if __name__ == "__main__":
    app = RoverBaseApp(None)
    app.title("Rover ICR Projection App")
    app.mainloop()