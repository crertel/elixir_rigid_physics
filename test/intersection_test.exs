defmodule ElixirRigidPhysicsTest.IntersectionTest do
  use ExUnit.Case, async: true
  doctest ElixirRigidPhysics.Collision.Intersection.SphereSphere
  doctest ElixirRigidPhysics.Collision.Intersection.SphereCapsule
  #doctest ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule
end
