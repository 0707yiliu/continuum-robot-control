import torch
import torch.nn as nn
import torch.optim as optim
import matplotlib.pyplot as plt
import numpy as np

class SimpleNet(nn.Module):
    def __init__(self):
        super(SimpleNet, self).__init__()
        self.model = nn.Sequential(
            nn.Linear(2, 20),    
            nn.ReLU(),
            nn.Linear(20, 20),  
            nn.ReLU(),
            nn.Linear(20, 1)   
        )

    def forward(self, x):
        return self.model(x)

x_vel_np = np.load('/home/yi/rosbags/continuum_demonstration/rosbag_all_topic_20250519_192818/NNinput_joint0_vel.npy')
x_absjoint_np = np.load('/home/yi/rosbags/continuum_demonstration/rosbag_all_topic_20250519_192818/NNinput_abs_joint0.npy')
y_np = np.load('/home/yi/rosbags/continuum_demonstration/rosbag_all_topic_20250519_192818/NNoutput.npy')



# plt.plot(x_np, label="input")
# plt.plot(y_np, label="output")
# # plt.plot(x_np, predicted, label="Predicted")
# plt.legend()
# plt.title("training data input-output")
# plt.show()



x_np = np.vstack((x_absjoint_np, x_vel_np)).T
y_np = y_np.reshape(-1, 1)
print(x_np.shape, y_np.shape)
x = torch.from_numpy(x_np.astype(np.float32))
y = torch.from_numpy(y_np.astype(np.float32))

model = SimpleNet()
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.01)

for epoch in range(1000):
    optimizer.zero_grad()
    outputs = model(x)
    loss = criterion(outputs, y)
    loss.backward()
    optimizer.step()
    if epoch % 50 == 0:
        print(f"Epoch {epoch}, Loss: {loss.item():.4f}")

torch.save(model.state_dict(), "mappingNN_absjoint_current_model.pth")

predicted = model(x).detach().numpy()

plt.plot(x_np, label="True input")
plt.plot(y_np, label="True output")
plt.plot(predicted, label="Predicted output")
plt.legend()
plt.title("Simple Neural Network Fit")
plt.show()
