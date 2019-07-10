defmodule ElixirRigidPhysics.Geometry do
  @moduledoc """
  Module for union of geometry types.
  """
  alias ElixirRigidPhysics.Geometry.Sphere
  alias ElixirRigidPhysics.Geometry.Capsule
  alias ElixirRigidPhysics.Geometry.Box
  alias ElixirRigidPhysics.Geometry.Plane

  @type geometry :: Sphere.sphere() | Capsule.capsule() | Box.box() | Plane.plane()
end
