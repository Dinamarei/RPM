from flask import Flask, render_template, request, flash
import ast
import geocoder

app = Flask(_name_, template_folder='/data/data/com.termux/files')


def test():
    x1 = 30.0
    x2 = 32.0
    y1 = 29.0
    y2 = 31.0
    hr = 0
    x = ""
    mysos = ""
    temp = "heart rate: 125"
    #temp2=["125","120"]
    war = "out of bounds"
    hf = "critical condition"
    with open('/data/data/com.termux/files/home/storage/shared/Logserial/serial_20211213_070243.txt', 'r') as f:
       lines = f.readlines()
       l = lines[-4:]  # get last 3 lines

       for i in l:
           if i[0] == 'H':
              # temp2=i.strip().split(':')
               hr = i.replace("Heart rate: ", "")
               hr = hr.replace("\n", "")
           if i[0] == 'S':
               mysos = "SOS"
           else:
               mysos = ""

       g = geocoder.ip('me')
       lt = float(g.lat)
       lg = float(g.lng)

    for line in l:
        x = x + str(line) + str(g.latlng)

    #x=x+hf

    if lt < y1 or lt > y2 or lg < x1 or lg > x2:
        x = x + war
    if int(hr) > 5:
        mysos2 = "Critical Heart Rate"
    else:
        mysos2 = ""

    return hr, g.latlng, mysos, mysos2


@app.route('/')
def index():
    txt1, txt2, txt3, txt4 = test()
    return render_template('myhtml.html', text1=txt1, text2=txt2, text3=txt3)


if _name_ == '_main_':
    app.run()
