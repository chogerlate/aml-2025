# 1. define scaling function
from typing import Tuple
def scale(data, from_interval: Tuple[float, float], to_interval: Tuple[float, float]=(0, 1)):
    from_min, from_max = from_interval
    to_min, to_max = to_interval
    scaled_data = to_min + (data - from_min) * (to_max - to_min) / (from_max - from_min)
    return scaled_data


# 2. read data
import pandas as pd
from matplotlib import pyplot as plt
dataframe = pd.read_csv('history.csv', sep=',')
dataframe.iloc[:, 0:8] = scale(dataframe.iloc[:, 0:8], from_interval=(0, 100), to_interval=(0,1))
dataframe.iloc[:, 8] = scale(dataframe.iloc[:, 8], from_interval=(-180, 180), to_interval=(0,1))
dataframe.iloc[:, 9:] = scale(dataframe.iloc[:, 9:], from_interval=(-5, 5), to_interval=(0,1))


# 3. select input and output of the ANN
x = dataframe.iloc[:, :9]
y = dataframe.iloc[:, 9:]


from keras.models import Sequential
from keras.layers import Dense
# Use AdamW optimizer for better convergence
from keras.optimizers import AdamW
from keras.callbacks import EarlyStopping, ReduceLROnPlateau

def create_model(input_shape=(9,)):
    """Create neural network model with dropout for regularization."""
    model = Sequential([
        Dense(32, activation='relu', input_shape=input_shape),
        Dense(16, activation='relu'),
        Dense(2, activation='sigmoid')
    ])
    
    model.compile(
        optimizer=AdamW(learning_rate=0.001),
        loss='mean_squared_error',
        metrics=['mae']
    )
    
    return model

model = create_model()
print("Model created with dropout layers for regularization")
model.summary()

# callbacks and validation
callbacks = [
    EarlyStopping(monitor='val_loss', patience=15, restore_best_weights=True, verbose=1),
    ReduceLROnPlateau(monitor='val_loss', factor=0.5, patience=5, min_lr=1e-6, verbose=1)
]

history = model.fit(
    x, y,
    batch_size=512,
    epochs=1000,
    shuffle=True,
    validation_split=0.2,
    callbacks=callbacks,
    verbose=1
)

model.save('assignment5_model.h5')

# plots: loss and MAE with validation
plt.figure(figsize=(12, 4))
# Loss
plt.subplot(1, 2, 1)
plt.plot(history.history['loss'], label='train loss')
plt.plot(history.history.get('val_loss', []), label='val loss')
plt.title('Model Loss')
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.legend()
# MAE
plt.subplot(1, 2, 2)
plt.plot(history.history.get('mae', []), label='train MAE')
plt.plot(history.history.get('val_mae', []), label='val MAE')
plt.title('Model MAE')
plt.xlabel('Epoch')
plt.ylabel('MAE')
plt.legend()
plt.tight_layout()
# Save figures and history
plt.savefig('training_curves.png', dpi=150)
plt.show()