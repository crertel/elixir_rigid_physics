defmodule ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule do
  @moduledoc """
  Module for sphere-capsule intersection tests.
  """

  require ElixirRigidPhysics.Dynamics.Body, as: Body
  require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
  require ElixirRigidPhysics.Collision.ContactManifold, as: ContactManifold
  require ElixirRigidPhysics.Collision.ContactPoint, as: ContactPoint

  alias ElixirRigidPhysics.Geometry.Util, as: GUtil
  alias Graphmath.Vec3
  alias Graphmath.Quatern

  @verysmol 1.0e-12

  @doc """
  Check the intersection of two capsules.

  ## Examples
    iex> # Check non-touching capsules
    iex> alias ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> a = Body.body( shape: Capsule.capsule(axial_length: 1, cap_radius: 0.5), position: {0.0, 0.0, 0.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 1, cap_radius: 0.5), position: {5.0, 0.0, 0.0})
    iex> Intersection.check(a,b)
    :no_intersection

    iex> # Check coincident capsules
    iex> alias ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> a = Body.body( shape: Capsule.capsule(axial_length: 1, cap_radius: 0.5), position: {0.0, 0.0, 0.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 1, cap_radius: 0.5), position: {0.0, 0.0, 0.0})
    iex> Intersection.check(a,b)
    :coincident

    iex> # Check grazing cap contact
    iex> alias ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> a = Body.body( shape: Capsule.capsule(axial_length: 3.0, cap_radius: 0.5), position: {0.0, 0.0, 0.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 2.0, cap_radius: 1.0), position: {0.0, 4.0, 0.0})
    iex> Intersection.check(a,b)
    {:contact_manifold, {{:contact_point, {0.0, 2.0, 0.0}, 1.0}}, {0.0, 1.0, 0.0}}

  """
  def check(
        Body.body(shape: Capsule.capsule(cap_radius: cr_a), position: p_a, orientation: o_b),
        Body.body(shape: Capsule.capsule(cap_radius: cr_b) = c, position: p_b, orientation: o_b)
      ) do
    :ok
  end
end
