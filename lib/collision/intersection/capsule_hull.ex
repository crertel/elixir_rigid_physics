defmodule ElixirRigidPhysics.Collision.Intersection.CapsuleHull do
  @moduledoc """
  Module for sphere-hull intersection tests.
  """

  require ElixirRigidPhysics.Dynamics.Body, as: Body
  require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
  require ElixirRigidPhysics.Geometry.Hull, as: Hull
  require ElixirRigidPhysics.Collision.Contact, as: Contact
  alias Graphmath.Vec3

  @verysmol 1.0e-12

  @doc """
  Tests intersections of two bodies.
  """
  @spec check(Body.body(), Body.body()) :: Contact.contact_result()
  def check(
        Body.body(
          shape: Capsule.capsule(axial_length: l_a, cap_radius: r_a),
          position: p_a,
          orientation: o_b
        ),
        Body.body(shape: Hull.hull(faces: faces), position: p_b, orientation: o_b)
      ) do
    :no_intersection
  end
end
