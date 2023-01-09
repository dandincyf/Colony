# coding=gbk
import communication_control_module.animation as ccm_ani
import communication_control_module.detection as ccm_odm
import numpy as np
import random
import matplotlib.pyplot as plt
import tkinter
import tkinter.messagebox
from tkinter import ttk
from tkinter import *
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib
matplotlib.use('TkAgg')
#import func

# f = plt.figure(num=2, figsize=(8, 6), dpi=80,
#                frameon=True)

def cmp(result):
    if result[0]>result[1]:
        tmp=result[0]
        result[0]=result[1]
        result[1]=tmp
    if result[0]>result[2]:
        tmp=result[0]
        result[0]=result[2]
        result[2]=tmp
    if result[1]>result[2]:
        tmp=result[1]
        result[1]=result[2]
        result[2]=tmp

class Page(object):
    def __init__(self, master=None):
        self.root = master
        root.title('Drone UI')
        self.task_flag = 0
        self.init_root()

    def init_root(self):
        sw = root.winfo_screenwidth()
        sh = root.winfo_screenheight()
        ww = 300
        wh = 300
        #root.geometry("%dx%d" % (ww, wh))
        root.geometry("%dx%d+%d+%d" % (ww, wh, (sw-ww)/2, (sh-wh)/2))
        root.resizable(False, False)
        self.rootPage()

    def rootPage(self):
        self.rootpage = Frame(self.root)
        self.rootpage.pack()
        btn1 = Button(self.rootpage, text="����Эͬ����", width=15,
                      height=2, command=self.rootPage_changePageTask)
        btn1.pack(pady=15)

        btn2 = Button(self.rootpage, text="�ٿط�Ⱥ", width=15,
                      height=2, command=self.rootPage_changePageCmd)
        btn2.pack(pady=5)

        btn3 = Button(self.rootpage, text="�鿴��Ⱥ��Ϣ", width=15,
                      height=2, command=self.rootPage_changePageInfo)
        btn3.pack(pady=15)
        # root.mainloop()

    def rootPage_changePageTask(self):
        self.rootpage.pack_forget()
        self.taskPage()

    def rootPage_changePageCmd(self):
        self.rootpage.pack_forget()
        self.cmdPage()

    def rootPage_changePageInfo(self):
        # self.rootpage.pack_forget()
        self.infoPage()

    def taskPage(self):
        self.taskpage = Frame(self.root)
        self.taskpage.pack()
        self.check_flag=0 #У�����
        label1 = Label(self.taskpage, text='��ѡ��Эͬ����',
                       width=15, height=3, font=20)
        label1.pack()
        self.comvalue = tkinter.StringVar()
        comboxlist = ttk.Combobox(
            self.taskpage, textvariable=self.comvalue, state="readonly")
        comboxlist["values"] = ("��ӷ���", "��������Ŀ��","��������")
        comboxlist.pack()

        btn1 = Button(self.taskpage, text="��������", width=10,
                      height=1, command=self.taskPage_doTask)
        btn2=Button(self.taskpage, text="���˻��Լ�", width=10,
                      height=1, command=self.taskPage_doCheck)
        btn3=Button(self.taskpage, text="���˻�У��", width=10,
                      height=1, command=self.taskPage_doAdjust)
        btn4 = Button(self.taskpage, text="����", width=10,
                      height=1, command=self.taskPage_doBack)
        btn1.pack(pady=5)
        btn2.pack(pady=10)
        btn3.pack(pady=10)
        btn4.pack(pady=10)

    def taskPage_doTask(self):
        if self.check_flag==0:
            tkinter.messagebox.showinfo(
                title='��ʾ��', message='��������˻��Լ죡')
        else:
            if self.comvalue.get() != "":
                if self.comvalue.get() == "��ӷ���":
                    # self.taskpage.pack_forget()
                    # self.formationFlying()
                    # self.fig=ccm_ani.evolution_formation()
                    self.task_flag = 1
                elif self.comvalue.get()=="��������Ŀ��":
                    self.task_flag = 2
                    # self.taskpage.pack_forget()
                    # self.obstacleAvoid()
                else:
                    self.task_flag=3
                result = tkinter.messagebox.showinfo(
                    title='��ʾ��', message='Эͬ���񷢲��ɹ���')
            else:

                result = tkinter.messagebox.showinfo(
                    title='��ʾ��', message='Эͬ���񷢲�ʧ�ܣ�')
            self.taskpage.pack_forget()
            self.rootPage()

    def taskPage_doCheck(self):
        if self.check_flag==1:
            tkinter.messagebox.showinfo(
                title='��ʾ��', message='���˻��������뷢��Эͬ����')
        else:
            result=random.sample(range(0,10),3)
            cmp(result)
            text='���˻�'+str(result[0])+","+str(result[1])+","+str(result[2])+"ָ�����쳣����У��"
            tkinter.messagebox.showinfo(
                title='��ʾ��', message=text)
    def taskPage_doAdjust(self):
        self.check_flag=1
        tkinter.messagebox.showinfo(
            title='��ʾ��', message='���˻���У��')
    def taskPage_doBack(self):
        self.taskpage.pack_forget()
        self.rootPage()

    def cmdPage(self):
        self.cmdpage = Frame(self.root)
        self.cmdpage.pack()
        notebook = ttk.Notebook(self.cmdpage, height=250, width=220)
        frame1 = Frame()
        frame2 = Frame()

        frame1_label = Label(frame1, text='��ѡ�������',
                             width=15, height=3, font=20)
        frame1_label.pack()
        self.frame1_comvalue1 = tkinter.StringVar()
        frame1_comboxlist = ttk.Combobox(
            frame1, textvariable=self.frame1_comvalue1, state="readonly")
        frame1_comboxlist["values"] = (
            "���", "����", "����", "����", "����", "��λת��")
        frame1_comboxlist.pack()

        frame1_btn1 = Button(frame1, text="ִ������", width=10,
                             height=1, command=self.cmdPage_frame1_doTask)
        frame1_btn2 = Button(frame1, text="����", width=10,
                             height=1, command=self.cmdPage_frame1_doBack)
        frame1_btn1.pack(pady=15)
        frame1_btn2.pack(pady=5)

        frame2_label1 = Label(frame2, text='��ѡ�����˻���',
                              width=15, height=2, font=10)
        frame2_label1.pack()
        self.frame2_comvalue1 = tkinter.StringVar()
        frame2_comboxlist = ttk.Combobox(
            frame2, textvariable=self.frame2_comvalue1, state="readonly")
        frame2_comboxlist["values"] = (
            "1", "2", "3", "4", "5", "6", "7", "8", "9", "10")
        frame2_comboxlist.pack()

        frame2_label2 = Label(frame2, text='��ѡ�������',
                              width=15, height=2, font=10)
        frame2_label2.pack()
        self.frame2_comvalue2 = tkinter.StringVar()
        frame2_comboxlist = ttk.Combobox(
            frame2, textvariable=self.frame2_comvalue2, state="readonly")
        frame2_comboxlist["values"] = (
            "���", "����", "����", "����", "����", "��λת��")
        frame2_comboxlist.pack()

        frame2_btn1 = Button(frame2, text="ִ������", width=10,
                             height=1, command=self.cmdPage_frame2_doTask)
        frame2_btn2 = Button(frame2, text="����", width=10,
                             height=1, command=self.cmdPage_frame2_doBack)
        frame2_btn1.pack(pady=10)
        frame2_btn2.pack(pady=3)

        notebook.add(frame1, text='�ٿط�Ⱥ')
        notebook.add(frame2, text='�ٿ����˻�')
        notebook.pack()

    def cmdPage_frame1_doTask(self):
        result = tkinter.messagebox.showinfo(
            title='��ʾ��', message='frame1������ɹ���')
        self.cmdpage.pack_forget()
        self.rootPage()

    def cmdPage_frame1_doBack(self):
        self.cmdpage.pack_forget()
        self.rootPage()

    def cmdPage_frame2_doTask(self):
        result = tkinter.messagebox.showinfo(
            title='��ʾ��', message='frame2������ɹ���')
        self.cmdpage.pack_forget()
        self.rootPage()

    def cmdPage_frame2_doBack(self):
        self.cmdpage.pack_forget()
        self.rootPage()

    def infoPage(self):
        if self.task_flag == 0:
            result = tkinter.messagebox.showinfo(
                title='��ʾ��', message='��Ⱥδ�������뷢��Эͬ�����ٿط�Ⱥ��ɣ�')
            # self.rootPage()
        else:
            if self.task_flag == 1:
                ccm_ani.flyShow()
            elif self.task_flag==2:
                ccm_odm.object_detection_mission()
            elif self.task_flag==3:
                ccm_ani.avoid_obstacles_animation()
            self.task_flag = 0

    def infoPage_doBack(self):
        self.infopage.pack_forget()
        self.init_root()


if __name__ == "__main__":
    root = Tk()
    Page(root)
    root.mainloop()





