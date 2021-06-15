defmodule ElixirRigidPhysics.Collision.Narrowphase do
  @moduledoc """
  Functions for generating collision manifolds for body pairs.

  Supported:
  * sphere-sphere
  * sphere-capsule
  * capsule-capsule

  Planned:
  * sphere-hull
  * capsule-hull
  * hull-hull

  Check [here](http://media.steampowered.com/apps/valve/2015/DirkGregorius_Contacts.pdf) for a good summary of things.
  """

  require ElixirRigidPhysics.Dynamics.Body, as: Body
  require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
  require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
  require ElixirRigidPhysics.Geometry.Hull, as: Hull
  require ElixirRigidPhysics.Collision.Contact, as: Contact

  alias ElixirRigidPhysics.Collision.Intersection.SphereSphere
  alias ElixirRigidPhysics.Collision.Intersection.SphereCapsule
  alias ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule
  alias ElixirRigidPhysics.Collision.Intersection.SphereHull
  alias ElixirRigidPhysics.Collision.Intersection.CapsuleHull
  alias ElixirRigidPhysics.Collision.Intersection.HullHull

  @doc """
  Tests the intersection of two shapes.
  """
  @spec test_intersection(Body.body(), Body.body()) :: Contact.contact_result()

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
        Body.body(shape: Sphere.sphere()) = a,
        Body.body(shape: Hull.hull()) = b
      ),
      do: SphereHull.check(a, b)

  def test_intersection(
        Body.body(shape: Hull.hull()) = b,
        Body.body(shape: Sphere.sphere()) = a
      ),
      do: SphereHull.check(a, b)

  def test_intersection(
        Body.body(shape: Capsule.capsule()) = a,
        Body.body(shape: Capsule.capsule()) = b
      ),
      do: CapsuleCapsule.check(a, b)

  def test_intersection(
        Body.body(shape: Capsule.capsule()) = a,
        Body.body(shape: Hull.hull()) = b
      ),
      do: CapsuleHull.check(a, b)

  def test_intersection(
        Body.body(shape: Hull.hull()) = b,
        Body.body(shape: Capsule.capsule()) = a
      ),
      do: CapsuleHull.check(a, b)

  def test_intersection(
        Body.body(shape: Hull.hull()) = a,
        Body.body(shape: Hull.hull()) = b
      ),
      do: HullHull.check(a, b)

  def test_intersection(_, _), do: {:error, :bad_bodies}
end
