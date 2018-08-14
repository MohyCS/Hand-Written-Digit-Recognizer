from tensorflow.examples.tutorials.mnist import input_data
import numpy as np

## makes sure these imports point to correct directory and correct files
import tf_classify_sensed_image as emc              # evaluation file, takes one image in 28x28 and outputs class
import tf_train_model_ASSIGNMENT_FILE as tm         # file we created in recitaiton

# get mnist as example
mnist = input_data.read_data_sets("MNIST_data/", one_hot=True)

# model number
model=8

## code calls the training file we created in recitation to train model
model_train = tm.build_train()                  ### comment line out for evaluation only
model_train.build_train_network(model)          ### comment line out for evaluation only

# get single mnist image for evaluation
batch_xs, batch_ys = mnist.test.next_batch(1)

# code opens saved trained model created by code above
model_eval = emc.evaluate_model()
image = model_eval.evaluate_model(model_version=model,input=batch_xs)

# prints out classification and correct label
print('Classification:')
print(image)
print('Correct Label:')
print(np.where(batch_ys==1)[1])