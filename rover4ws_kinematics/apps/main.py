import tkinter as tk
from .secondary import SecondApp


class MainApp(tk.Tk):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.title("Rover ICR Projection App")
        self.parent = parent
        self.generateWidgets()


    
    def generateWidgets(self):
        self.geometry('600x150')
        self.eval('tk::PlaceWindow . center')
        self.grid()
        self.main_label = tk.Label(text="Four Wheel Stering Rover ICR App v1.0.1", font=80)
        self.main_label.grid(row=0,column=200)
        
        self.options = tk.LabelFrame(self, text="Control Options")
        self.options.grid(row=10, column=0)
        self.check_boxes = []
        self.check_boxes_vals = []
        self._generateCheckButtonsWidgets()

        self.start_button = tk.Button(text='Start',fg="green", command=self._startButtonCallback)
        self.start_button.grid(row=300,column=400)
        self.quit_button = tk.Button(text="Quit",fg="red", command=lambda:self.destroy())
        self.quit_button.grid(row=300,column=600)
    def _generateCheckButtonsWidgets(self):
        self._buttons_initialized = False
        buttons = 2
        labels = ['Keyboard Control', 'Mouse Control']

        for button in range(buttons):
            self.check_boxes_vals.append(tk.BooleanVar())
            self.check_boxes.append(tk.Checkbutton(self.options, text=labels[button],
            variable=self.check_boxes_vals[button], command=lambda id=button: self._check_but_callback(id), onvalue=True,offvalue=False))
            self.check_boxes[button].grid(row=button+5, column=0,pady=10)
            #self.check_boxes[button].bind(button, self._check_but_callback)
        self._buttons_initialized = True
    
    def _check_but_callback(self,event):
        if self._buttons_initialized:
            for i in range(len(self.check_boxes_vals)):
                if i != event:
                    self.check_boxes_vals[i].set(False)
        else:
            pass

    def _startButtonCallback(self):
        self.withdraw()
        self.second_app = SecondApp(calling_app=self)
        self.second_app.run()


    

    def run(self):
        self.mainloop()

    


if __name__ == '__main__':
    app = MainApp()
    app.run()
