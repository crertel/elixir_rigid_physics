defmodule ElixirRigidPhysics.Dynamics do
  alias ElixirRigidPhysics.World

  def step( %World{timestep: timestep, current_time: current_time} = world, dt) do
    new_world = %World{world |
    timestep: timestep + 1,
    current_time: current_time + dt
  }
  new_world
  end
end
