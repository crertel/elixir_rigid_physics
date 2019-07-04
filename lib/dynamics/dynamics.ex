defmodule ElixirRigidPhysics.Dynamics do
  @moduledoc """
  Dynamics module responsible for handling physics stepping and substepping of a world.
  """
  alias ElixirRigidPhysics.World
  alias ElixirRigidPhysics.Collision.Broadphase

  alias Graphmath.Vec3
  alias Graphmath.Quatern

  @spec step(World.t(), number) :: World.t()
  def step(
        %World{
          timestep: timestep,
          current_time: current_time,
          bodies: bodies,
          broadphase_acceleration_structure: old_acc_struct
        } = world,
        dt
      ) do
    import ElixirRigidPhysics.Dynamics.Body

    acc_struct = Broadphase.populate_acceleration_structure_from_bodies(old_acc_struct, bodies)

    # IO.inspect(Broadphase.generate_potential_colliding_pairs(acc_struct))

    new_bodies =
      for {r,
           body(
             position: position,
             orientation: orientation,
             linear_velocity: linear_velocity,
             angular_velocity: angular_velocity
           ) = b} <- bodies,
          into: %{} do
        # integrate linear velocity
        new_position = Vec3.scale(linear_velocity, dt) |> Vec3.add(position)

        # integrate angular velocity
        new_orientation = Quatern.integrate(orientation, angular_velocity, dt)

        new_body = body(b, position: new_position, orientation: new_orientation)

        {r, new_body}
      end

    %World{
      world
      | timestep: timestep + 1,
        current_time: current_time + dt,
        bodies: new_bodies,
        broadphase_acceleration_structure: acc_struct
    }
  end
end
