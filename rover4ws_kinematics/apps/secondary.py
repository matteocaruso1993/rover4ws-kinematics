import tkinter as tk
import matplotlib
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from ..src.utils.icr_handler import IcrHandler


class SecondApp(tk.Tk):
    def __init__(self, calling_app,parent=None) -> None:
        super().__init__(parent)
        self.title("Rover ICR Visualization App")
        self.parent = parent
        self.calling_app = calling_app
        self.generateWidgets()
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.initializeExternalLib()
        self._generateCheckButtonsWidgets()
        self.initializeChart()
        self.registered_icr = [0,0]
        self.input_angles = 4*[0]
        self._chart_update_period = self.slider_update.get()
        print(self._chart_update_period)
        self.visualization_frame.after(self._chart_update_period, self.updateChart)


    def generateWidgets(self):
        self.geometry('600x770')
        self.eval('tk::PlaceWindow . center')
        #self.grid()
        self.options = tk.LabelFrame(self, text="Drive Options")
        
        self.options.pack(side=tk.TOP,fill=tk.X, ipady=20)
        
        self.check_boxes = []
        self.check_boxes_vals = []

    def initializeChart(self):
        self.visualization_frame = tk.LabelFrame(self,text="Visualization")
        self.visualization_frame.pack(side=tk.TOP,fill=tk.BOTH)
        self.navi_frame = tk.LabelFrame(self,text="Navigation")
        self.navi_frame.pack(side=tk.TOP,fill=tk.X)
        self.fig = Figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.plot([],[])
        self.canv = FigureCanvasTkAgg(self.fig, master = self.visualization_frame)
        self.canv.draw()
        def onclick(event):
            #print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
            #    ('double' if event.dblclick else 'single', event.button,
            #    event.x, event.y, event.xdata, event.ydata))
            self.registered_icr = [event.xdata,event.ydata]
        

        cid = self.canv.mpl_connect('button_press_event', onclick)

        self.get_widz = self.canv.get_tk_widget()
        self.get_widz.pack(fill=tk.BOTH)

        self.toolbar = NavigationToolbar2Tk(self.canv, self.navi_frame)
        self.toolbar.pack(side=tk.BOTTOM,pady=10)
        #toolbar.update()
        self.slider_update = tk.Scale(self.navi_frame, from_=100, to=10000, orient=tk.HORIZONTAL,
                              command=self._updateFrequency, label="Update Period [ms]")
        self.slider_update.pack(side=tk.LEFT,ipadx=200)
    
    def _updateFrequency(self,val):
        self._chart_update_period = val

    def _generateCheckButtonsWidgets(self):
        labels = ['outer ackermann','inner ackermann','car like', 'symmetric ackermann','full_ackermann']
        row=0
        col=0
        for button in range(len(labels)):
            if row==2:
                row=0
                col+=1
            
            self.check_boxes_vals.append(tk.BooleanVar())
            if button==0:
                self.check_boxes_vals[button].set(True)
            self.check_boxes.append(tk.Checkbutton(self.options, text=labels[button],
            variable=self.check_boxes_vals[button], command=lambda id=button: self._check_but_callback(id), onvalue=True,offvalue=False))
            
            
            self.check_boxes[button].grid(row=row, column=col)
            #self.check_boxes[button].pack()
            row+=1
        self._buttons_initialized = True
        #self.check_boxes[0].select()

    def on_closing(self):
        if self.calling_app is not None:
            self.calling_app.deiconify()
        self._chart_update_period = float('inf')
        self.destroy()

    def run(self):
        self.mainloop()

    def _check_but_callback(self,event):
        if self._buttons_initialized:
            for i in range(len(self.check_boxes_vals)):
                print(event)
                if i == event:
                    self.current_mode = self.check_boxes[i]['text']
                    self.mode_pointer = i
                    self.icr_handler.setMode(self.icr_handler._valid_modes[i])
                else:
                    self.check_boxes_vals[i].set(False)
                    self.check_boxes[i].deselect()
        else:
            pass

    def updateChart(self):
        self.ax.clear()
        self.icr_handler.validateIcr(self.registered_icr)
        self.icr_handler.show(self.input_angles, fig=self.fig, ax=self.ax)
        self.canv.draw()
        #self.ax.axis('equal')
        self.visualization_frame.after(self._chart_update_period, self.updateChart)

    def initializeExternalLib(self):
        self.icr_handler = IcrHandler(mode='outer_ack')
        self.icr_handler.initialize()





if __name__ == '__main__':
    app = SecondApp(calling_app=None)
    app.run()
