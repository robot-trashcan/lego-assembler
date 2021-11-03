"""
This program utilizes a neural network to learn a mapping from servo angles to
x/y coordinates, and then inverts the model to calculate servo extensions from
a given set of x/y coordinates
"""

# Import libraries
import arm_model
import tqdm
import torch
import torch.nn as nn
from torch.utils.data import TensorDataset, DataLoader

# Constants
SAVE_PATH = './kinematics_inverse.ckpt'
device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
lr = 0.00003
batch_size = 2000
epochs = 1000
thresh = 0.1
train_model = True


# Helper functions
def load_model():
    return torch.load(SAVE_PATH)


def create_data(dof: int = 3, min_angle: float = -torch.pi / 2,
                max_angle: float = torch.pi / 2, num_samples: int = 100):
    # Get single stream
    single_stream = torch.linspace(min_angle, max_angle, num_samples).to(device)
    full_angles = torch.cartesian_prod(*[single_stream for _ in range(dof)])
    full_angles = torch.cumsum(full_angles, axis=1)

    # Convert to progressive sums
    base_angle = (torch.pi / 2 ) * torch.ones(full_angles.shape[0], 1).to(device)
    full_base_angles = torch.cat([base_angle, full_angles], axis=1)

    # Compute rotation vectors
    rotation_vecs = torch.stack([torch.cos(full_base_angles), torch.sin(full_base_angles)], axis=2)

    # Scale rotation vectors by distance
    joint_distances = torch.Tensor(arm_model.joint_distances).to(device).unsqueeze(0).unsqueeze(2)
    polar_vecs = rotation_vecs * joint_distances

    return full_angles, torch.sum(polar_vecs, axis=1)


def normalize_data(data, bounds):
    return (data - bounds[0]) / (bounds[1] - bounds[0])


def scale_data(data, bounds):
    return bounds[0] + data * (bounds[1] - bounds[0])


# Model
class ArmKinematics(nn.Module):
    def __init__(self, in_range, out_range, dof: int = 3,
                 hidden_size: int = 20, out_dim: int = 2):
        super(ArmKinematics, self).__init__()

        # Save parameters
        self.dof = dof
        self.hidden_size = hidden_size
        self.out_dim = out_dim
        self.in_range = in_range
        self.out_range = out_range

        # Define layers
        self.layers = nn.Sequential(
            nn.Linear(dof, hidden_size),
            nn.Tanh(),
            nn.Linear(hidden_size, hidden_size),
            nn.Tanh(),
            nn.Linear(hidden_size, out_dim),
            nn.Sigmoid(),
        )

    def forward(self, X):
        return self.layers(X)

    def scaled_forward(self, X):
        normed_x = normalize_data(X, self.in_range)
        out = self.layers(X)
        return scale_data(out, self.out_range)


def network_inverse(model, target, loss_fn, lr, thresh, max_iterations):
    # Create input
    in_tensor = torch.zeros(1, model.dof, device=device, 
                            requires_grad=True)
    loss_val = thresh + 1
    num_iterations = 0

    # Turn off gradient for model
    for param in model.parameters():
        param.requires_grad = False

    # Create optimizer
    optim = torch.optim.Adam([in_tensor], lr=lr)

    # Progressively narrow
    while loss_val > thresh and num_iterations < max_iterations:
        # Forward pass
        out = model.scaled_forward(in_tensor)
        loss_val = loss_fn(out, target)

        # Backward pass
        optim.zero_grad()
        loss_val.backward()
        optim.step()

        num_iterations += 1

    return in_tensor


if __name__ == '__main__':
    # Load the dataset
    angles, displacements = create_data(num_samples=100)

    # Normalize the dataset
    angle_min = torch.min(angles) # -4.7124
    angle_max = torch.max(angles) # 4.7124
    displacement_min = torch.min(displacements) # -9.6558
    displacement_max = torch.max(displacements) # 17.2808
    angles = (angles - angle_min) / (angle_max - angle_min)
    displacements = (displacements - displacement_min) / (displacement_max - displacement_min)
    angles, displacements = displacements, angles

    # Create PyTorch dataset and loader
    dataset = TensorDataset(angles, displacements)
    loader = DataLoader(dataset, batch_size=batch_size,
                        shuffle=True)

    # Initialize model and optimizer
    # model = ArmKinematics((angle_min, angle_max), (displacement_min, displacement_max)).to(device)
    model = ArmKinematics((angle_min, angle_max), (displacement_min, displacement_max)).to(device)
    model = ArmKinematics((displacement_min, displacement_max), (angle_min, angle_max), dof=2, out_dim=3)
    optim = torch.optim.Adam(model.parameters(), lr=lr)
    loss = nn.MSELoss()

    if train_model:
        # Train the model
        for epoch in range(epochs):
            running_loss = 0

            for angle_batch, displacement_batch in tqdm.tqdm(loader):
                # Cast to device
                angle_batch = angle_batch.to(device)
                displacement_batch = displacement_batch.to(device)

                # Forward
                out = model(angle_batch)

                # Compute loss
                loss_val = loss(displacement_batch, out)

                # Backprop
                optim.zero_grad()
                loss_val.backward()
                optim.step()

                # Save
                running_loss += loss_val.detach().item()

            print(f'Epoch {epoch+1}/{epochs}, Loss: {running_loss}')

        torch.save(model, SAVE_PATH)

    else:
        model = load_model().to(device)
