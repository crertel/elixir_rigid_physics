defmodule ElixirRigidPhysics.Collision.Narrowphase do
  @moduledoc """
  Functions for generating collision manifolds for body pairs.

  Supported:
  * sphere-sphere
  * sphere-capsule

  Planned:
  * capsule-capsule
  * box-sphere
  * capsule-box

  Check [here](http://media.steampowered.com/apps/valve/2015/DirkGregorius_Contacts.pdf) for a good summary of things.
  """

  require ElixirRigidPhysics.Dynamics.Body, as: Body
  require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
  require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
  require ElixirRigidPhysics.Collision.ContactManifold, as: ContactManifold

  alias ElixirRigidPhysics.Collision.Intersection.SphereSphere
  alias ElixirRigidPhysics.Collision.Intersection.SphereCapsule

  @doc """
  Tests the intersection of two shapes.
  """
  @spec test_intersection( Body.body, Body.body) :: ContactManifold.contact_manifold()
  def test_intersection( Body.body(shape: Sphere.sphere()) = a, Body.body(shape: Sphere.sphere()) = b),
    do: SphereSphere.check(a, b)

  def test_intersection( Body.body(shape: Sphere.sphere()) = a, Body.body(shape: Capsule.capsule()) = b),
    do: SphereCapsule.check(a, b)
end

# iex> # Check non-touching capsules
#     iex> alias ElixirRigidPhysics.Collision.Intersection
#     iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
#     iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
#     iex> a = Body.body( shape: Capsule.capsule(axial_length: 1, cap_radius: 0.5), position: {0.0, 0.0, 0.0})
#     iex> b = Body.body( shape: Capsule.capsule(axial_length: 1, cap_radius: 0.5), position: {5.0, 0.0, 0.0})
#     iex> Intersection.test_intersection(a,b)
#     :no_intersection

#     iex> # Check coincident capsules
#     iex> alias ElixirRigidPhysics.Collision.Intersection
#     iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
#     iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
#     iex> a = Body.body( shape: Capsule.capsule(axial_length: 1, cap_radius: 0.5), position: {0.0, 0.0, 0.0})
#     iex> b = Body.body( shape: Capsule.capsule(axial_length: 1, cap_radius: 0.5), position: {0.0, 0.0, 0.0})
#     iex> Intersection.test_intersection(a,b)
#     :coincident

#     iex> # Check grazing cap contact
#     iex> alias ElixirRigidPhysics.Collision.Intersection
#     iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
#     iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
#     iex> a = Body.body( shape: Capsule.capsule(axial_length: 3.0, cap_radius: 0.5), position: {0.0, 0.0, 0.0})
#     iex> b = Body.body( shape: Capsule.capsule(axial_length: 2.0, cap_radius: 1.0), position: {0.0, 4.0, 0.0})
#     iex> Intersection.test_intersection(a,b)
#     {:contact_manifold, {{:contact_point, {0.0, 2.0, 0.0}, 1.0}}, {0.0, 1.0, 0.0}}
