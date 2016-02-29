import httplib
import re
import csv

host = '192.168.43.54' #has occasionally been '192.168.43.54'
conn = httplib.HTTPConnection(host)


while True:
    conn.request("GET", "/")
    respObj = conn.getresponse()
    readings = re.findall('\d+', respObj.read())
    fd = open('nofall.csv','a')
    w = csv.writer(fd)
    w.writerow([readings])

    fd.close()
    print readings

    #Apply machine learning to readings