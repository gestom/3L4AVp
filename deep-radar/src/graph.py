from matplotlib import pyplot as plt

history = []
with open("dump-80-2500.pickle", "rb") as f:
	history = pickle.load(f)

plt.plot(history['acc'])
plt.plot(history['val_acc'])
plt.title('model accuracy')
plt.ylabel('accuracy')
plt.xlabel('epoch')
plt.legend(['train', 'val'], loc='upper left')
plt.show()