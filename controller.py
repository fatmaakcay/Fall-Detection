from __future__ import unicode_literals
from __future__ import print_function

import csv
import re
import serial

from play import play_sound
from train_ml import preprocess_data, train_data
    
# Train ML models
row_length = 38
data_types = preprocess_data(row_length)
classifiers = train_data(data_types)

# Set up reading mechanism
ser = serial.Serial()
ser.baudrate = 9600
ser.port = '/dev/cu.usbmodem1421'
ser.open()

# Set up ML prediction
prior_row = None
prior_row_2x = None

while True:
    readings = ser.readline()
    data = re.findall('\d+', readings)
    if len(data) == row_length:
        if prior_row != None:
            with open('predictions.csv', 'a') as csvfile:        
                writer = csv.writer(csvfile)
                kernels = ['linear', 'poly']
                for kernel in kernels:                
                    classifier = classifiers['doubles'][kernel]
                    prediction = classifier.predict(prior_row + data)
                    print(('doubles', kernel, prediction))
                    assert prediction.shape == (1,)                
                    if prediction[0] == 1:
                        print("FALL DETECTED")
                        play_sound()
                    writer.writerow(['doubles', kernel, prediction])
                """for kernel_type, classifier in classifiers['doubles'].items():
                    prediction = classifier.predict(prior_row + data)
                    print(('doubles', kernel_type, prediction))
                    assert prediction.shape == (1,)                
                    if prediction[0] == 1:
                        print("FALL DETECTED")
                        play_sound()
                    writer.writerow(['doubles', kernel_type, prediction])
                if prior_row_2x != None:
                    for kernel_type, classifier in classifiers['triples'].items():
                        prediction = classifier.predict(prior_row_2x + prior_row + data)                
                        print(('triples', kernel_type, prediction))
                        assert prediction.shape == (1,)                
                        if prediction[0] == 1:
                            print("FALL DETECTED")
                            play_sound()
                        writer.writerow(['triples', kernel_type, prediction])"""
        prior_row_2x = prior_row
        prior_row = data
    else:
        print("line too short: " + str(len(data)))
    with open('test_data.csv','a') as csvfile:
        w = csv.writer(csvfile)
        w.writerow(data)