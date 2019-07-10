defmodule ElixirRigidPhysics.Collision.Broadphase do
  @moduledoc """
  Module to handle broadphase collision detection.

  Broadphase collision detection is when we check all of the bodies to see which ones _likely_ are colliding.
  This information is then used to reduce the number of expensive checks we do in the narrowphase.

  This module is also the current home of the acceleration structures for the broadphase, even though that's going to change.
  """
  alias ElixirRigidPhysics.Util
  alias ElixirRigidPhysics.Collision.AABB
  alias ElixirRigidPhysics.Dynamics.Body

  require Record
  Record.defrecord(:broadphase_acc_struct, struct: [], bodies: [])

  @type body_acc_tuple :: {reference, Body.body(), AABB.aabb()}
  @type broadphase_acc_struct ::
          record(:broadphase_acc_struct, struct: [any], bodies: [body_acc_tuple])

  @spec create_acceleration_structure() :: broadphase_acc_struct
  def create_acceleration_structure() do
    broadphase_acc_struct()
  end

  @spec populate_acceleration_structure_from_bodies(broadphase_acc_struct, [
          {reference, Body.body()}
        ]) :: broadphase_acc_struct
  def populate_acceleration_structure_from_bodies(acc_struct, bodies) do
    new_bodies =
      for {ref, body} <- bodies do
        {ref, body, ElixirRigidPhysics.Collision.AABB.create_world_from_body(body)}
      end

    IO.inspect(new_bodies, lavel: "ACC BODIES")

    broadphase_acc_struct(acc_struct,
      bodies: new_bodies
    )
  end

  @spec generate_potential_colliding_pairs(broadphase_acc_struct) :: [
          {{reference, Body.body(), AABB.aabb()}, {reference, Body.body(), AABB.aabb()}}
        ]
  def generate_potential_colliding_pairs(broadphase_acc_struct(bodies: bodies)) do
    Util.List.generate_pairs(bodies, fn {_ref_a, _body_a, aabb_a}, {_ref_b, _body_b, aabb_b} ->
      AABB.overlaps?(aabb_a, aabb_b)
    end)
  end
end
