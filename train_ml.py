from __future__ import unicode_literals
from __future__ import print_function

import csv
import numpy as np
import re
from sklearn.svm import SVC

#process training data into form usable by sklearn

def preprocess_data(row_length):
    training_files = ['fall1', 'fall2', 'fall3', 'fall4', 'env', 'bend', 'step_out', 'stand']

    doubles_x = []
    #determined by manual inspection of test data
    doubles_fall_indices = [3, 4, 5, 6, 7, 16, 17, 18, 27, 39]

    triples_x = []
    #determined by manual inspection of test data
    triples_fall_indices = [2, 3, 4, 5, 6, 7, 14, 15, 16, 17, 24, 25, 35, 36]

    for training_file in training_files:
        prior_row = None
        prior_row_2x = None
        filename = 'training_data_1/' + training_file + '.csv'
        with open(filename, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                assert len(row) == 1                
                data = re.findall('\d+', row[0])
                if len(data) == row_length:
                    if prior_row != None:
                        doubles_x.append(prior_row + data)
                        if prior_row_2x != None:
                            triples_x.append(prior_row_2x + prior_row + data)
                    prior_row_2x = prior_row
                    prior_row = data
            
    doubles_x = np.array(doubles_x)
    triples_x = np.array(triples_x)

    doubles_y = np.zeros(doubles_x.shape[0])
    for i in doubles_fall_indices:
        doubles_y[i] = 1
    triples_y = np.zeros(triples_x.shape[0])
    for i in triples_fall_indices:
        triples_y[i] = 1

    return {'doubles': {'x': doubles_x, 'y': doubles_y}, 'triples': {'x': triples_x, 'y': triples_y}}

#Train SVM model with data
def train_data(data_types):
    classifiers = {}
    for data_type in data_types:
        classifiers[data_type] = {}
    kernel_types = [str('linear'), str('poly'), str('rbf'), str('sigmoid')]
    for data_type, arrays in data_types.items():
        for kernel_type in kernel_types:
            classifier = SVC(kernel=kernel_type)
            classifiers[data_type][kernel_type] = classifier.fit(arrays['x'], arrays['y'])
    return classifiers

#Predict using training data - really boring, change this when possible
def predict_original_data(classifiers, data_types):
    for data_type, data_type_classifiers in classifiers.items():
        for classifier_name, classifier in data_type_classifiers.items():
            print(data_type, classifier_name)
            print(classifier.predict(data_types[data_type]['x']))
