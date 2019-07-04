defmodule ElixirRigidPhysics.Geometry do
  @moduledoc """
  Module for union of geometry types.
  """
  alias ElixirRigidPhysics.Geometry.Sphere
  alias ElixirRigidPhysics.Geometry.Capsule
  alias ElixirRigidPhysics.Geometry.Box

  @type geometry :: Sphere.sphere() | Capsule.capsule() | Box.box()
end
