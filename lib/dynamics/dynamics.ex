defmodule ElixirRigidPhysics.Dynamics do
  alias ElixirRigidPhysics.World
  alias Graphmath.Vec3
  alias Graphmath.Quatern

  def step(%World{timestep: timestep, current_time: current_time, bodies: bodies} = world, dt) do

    new_bodies = for {r, b} <- bodies, into: %{} do

      %{
        position: position,
        orientation: orientation,

        linear_velocity: linear_velocity,
        angular_velocity: angular_velocity,
      } = b

      # integrate linear velocity
      new_position = Vec3.scale(linear_velocity, dt)|> Vec3.add(position)

      # integrate angular velocity
      new_orientation = Quatern.integrate(orientation, angular_velocity, dt)

      new_body = Map.merge(b, %{
        position: new_position,
        orientation: new_orientation
      })

      {r, new_body}
    end
    %World{world | timestep: timestep + 1, current_time: current_time + dt, bodies: new_bodies}
  end
end
