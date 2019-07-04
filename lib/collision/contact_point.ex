defmodule ElixirRigidPhysics.Collision.ContactPoint do
  @moduledoc """
  Module for contact points.

  Contacts points exist in a contact manifold, and give information about where two bodies intersect and how far.
  """
  alias Graphmath.Vec3

  require Record
  Record.defrecord(:contact_point, world_point: {0.0, 0.0, 0.0}, depth: 0.0)
  @type contact_point :: record(:contact_point, world_point: Vec3.vec3(), depth: number)
end
