defmodule ElixirRigidPhysics.Dynamics do
  @moduledoc """
  Dynamics module responsible for handling physics stepping and substepping of a world.
  """
  alias ElixirRigidPhysics.World
  alias ElixirRigidPhysics.Collision.Broadphase
  alias ElixirRigidPhysics.Collision.Narrowphase

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

    maybe_colliding_pairs = Broadphase.generate_potential_colliding_pairs(acc_struct)

    #IO.inspect(maybe_colliding_pairs, label: "MAYBE COLLIDING PAIRS")

    collisions =
      maybe_colliding_pairs
      |> Enum.reduce([], fn {{_a_ref, a_body, _a_aabb}, {_b_ref, b_body, _b_aabb}}, acc ->
        case Narrowphase.test_intersection(a_body, b_body) do
          :coincident -> acc
          :no_intersection -> acc
          {:error, _} -> acc
          manifold -> [manifold | acc]
        end
      end)

    #IO.inspect(collisions, label: "COLLIDING PAIRS")

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
        new_position = linear_velocity |> Vec3.scale(dt) |> Vec3.add(position)

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
        broadphase_acceleration_structure: acc_struct,
        collisions: collisions
    }
  end
end
