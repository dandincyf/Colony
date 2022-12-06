import tkinter

window = tkinter.Tk()
#设置窗口title
window.title('无人机仿真平台')
#设置窗口大小
window.geometry('1200x600')

# pack布局
l1 = tkinter.Label(window, text='用户名', bg='pink', font=('Arial',12), width=15,height=2)
l1.place(x=40, y=30, width=100, height=30)

# place布局
l2 = tkinter.Label(window, text='密码', bg='pink',font=('Arial',12), justify=tkinter.RIGHT, width=15,height=2)
l2.place(x=40, y=70, width=100, height=30)




