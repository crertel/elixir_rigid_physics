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
  alias ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule

  @doc """
  Tests the intersection of two shapes.
  """
  @spec test_intersection(Body.body(), Body.body()) ::
          ContactManifold.contact_manifold() | :no_intersection | :coincident
  def test_intersection(
        Body.body(shape: Sphere.sphere()) = a,
        Body.body(shape: Sphere.sphere()) = b
      ),
      do: SphereSphere.check(a, b)

  def test_intersection(
        Body.body(shape: Sphere.sphere()) = a,
        Body.body(shape: Capsule.capsule()) = b
      ),
      do: SphereCapsule.check(a, b)

  def test_intersection(
        Body.body(shape: Capsule.capsule()) = b,
        Body.body(shape: Sphere.sphere()) = a
      ),
      do: SphereCapsule.check(a, b)

  def test_intersection(
        Body.body(shape: Capsule.capsule()) = a,
        Body.body(shape: Capsule.capsule()) = b
      ),
      do: CapsuleCapsule.check(a, b)

  def test_intersection(_, _), do: {:error, :bad_bodies}
end
