defmodule ElixirRigidPhysics.Collision.Broadphase do
  alias ElixirRigidPhysics.Util
  alias ElixirRigidPhysics.Collision.AABB

  require Record
  Record.defrecord(:broadphase_acc_struct, struct: [], bodies: [])

  def create_acceleration_structure() do
    broadphase_acc_struct()
  end

  def populate_acceleration_structure_from_bodies(acc_struct, bodies) do
    new_bodies =
      for {ref, body} <- bodies do
        {ref, body, ElixirRigidPhysics.Collision.AABB.create_world_from_body(body)}
      end

    broadphase_acc_struct(acc_struct,
      bodies: new_bodies
    )
  end

  def generate_potential_colliding_pairs(broadphase_acc_struct(bodies: bodies)) do
    Util.List.generate_pairs(bodies, fn {_ref_a, _body_a, aabb_a}, {_ref_b, _body_b, aabb_b} ->
      AABB.overlaps?(aabb_a, aabb_b)
    end)
  end
end
