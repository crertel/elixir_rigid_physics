defmodule ElixirRigidPhysics.Collision.Intersection.HullHull do
  @moduledoc """
  Module for sphere-hull intersection tests.
  """

  require ElixirRigidPhysics.Dynamics.Body, as: Body
  require ElixirRigidPhysics.Geometry.Hull, as: Hull
  require ElixirRigidPhysics.Collision.ContactManifold, as: ContactManifold
  require ElixirRigidPhysics.Collision.ContactPoint, as: ContactPoint
  alias ElixirRigidPhysics.Collision.Narrowphase
  alias Graphmath.Vec3

  @verysmol 1.0e-12

  @doc """
  Tests intersections of two bodies.
  """
  @spec check(Body.body(), Body.body()) :: Narrowphase.contact_result
  def check(
        Body.body(shape: Hull.hull(faces: faces_a), position: p_a, orientation: o_b),
        Body.body(shape: Hull.hull(faces: faces_b), position: p_b, orientation: o_b)
      ) do
    :no_intersection
  end
end
