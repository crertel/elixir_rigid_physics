defmodule ElixirRigidPhysics.Collision.Narrowphase do
  @moduledoc """
  Functions for generating collision manifolds for body pairs.

  Supported:
  * sphere-sphere

  Planned:
  * sphere-capsule
  * box-sphere
  * capsule-box

  Check [here](http://media.steampowered.com/apps/valve/2015/DirkGregorius_Contacts.pdf) for a good summary of things.
  """
  require ElixirRigidPhysics.Collision.ContactManifold, as: ContactManifold
  require ElixirRigidPhysics.Collision.ContactPoint, as: ContactPoint

  require ElixirRigidPhysics.Dynamics.Body, as: Body
  require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
  require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
  # require ElixirRigidPhysics.Geometry.Box, as: Box

  alias ElixirRigidPhysics.Geometry.Util, as: GUtil

  alias Graphmath.Quatern
  alias Graphmath.Vec3

  @verysmol 1.0e-12

  # sphere-sphere

  @doc """
  Tests intersections of two bodies.

  ## Examples
    iex> # Check disjoint spheres
    iex> alias ElixirRigidPhysics.Collision.Narrowphase
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    iex> a = Body.body( shape: Sphere.sphere(radius: 2), position: {0.0, 0.0, 0.0})
    iex> b = Body.body( shape: Sphere.sphere(radius: 2) , position: {5.0, 0.0, 0.0})
    iex> Narrowphase.test_intersection(a,b)
    :no_intersection

    iex> # Check coincident spheres
    iex> alias ElixirRigidPhysics.Collision.Narrowphase
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    iex> a = Body.body( shape: Sphere.sphere(radius: 2), position: {0.0, 0.0, 0.0})
    iex> b = Body.body( shape: Sphere.sphere(radius: 2) , position: {0.0, 0.0, 0.0})
    iex> Narrowphase.test_intersection(a,b)
    :coincident

    iex> # Check grazing spheres of equal size
    iex> alias ElixirRigidPhysics.Collision.Narrowphase
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    iex> a = Body.body( shape: Sphere.sphere(radius: 1), position: {0.0, 0.0, 0.0})
    iex> b = Body.body( shape: Sphere.sphere(radius: 1) , position: {2.0, 0.0, 0.0})
    iex> Narrowphase.test_intersection(a,b)
    {:contact_manifold, {{:contact_point, {1.0, 0.0, 0.0}, 0.0}}, {1.0, 0.0, 0.0}}

    iex> # Check grazing spheres of different size
    iex> alias ElixirRigidPhysics.Collision.Narrowphase
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    iex> a = Body.body( shape: Sphere.sphere(radius: 1), position: {0.0, 0.0, 0.0})
    iex> b = Body.body( shape: Sphere.sphere(radius: 3) , position: {4.0, 0.0, 0.0})
    iex> Narrowphase.test_intersection(a,b)
    {:contact_manifold, {{:contact_point, {1.0, 0.0, 0.0}, 0.0}}, {1.0, 0.0, 0.0}}

    iex> # Check overlapping spheres of same size
    iex> alias ElixirRigidPhysics.Collision.Narrowphase
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    iex> a = Body.body( shape: Sphere.sphere(radius: 2), position: {0.0, 0.0, 0.0})
    iex> b = Body.body( shape: Sphere.sphere(radius: 2) , position: {3.0, 0.0, 0.0})
    iex> Narrowphase.test_intersection(a,b)
    {:contact_manifold, {{:contact_point, {1.5, 0.0, 0.0}, 1.0}}, {1.0, 0.0, 0.0}}

    iex> # Check overlapping spheres of different size
    iex> alias ElixirRigidPhysics.Collision.Narrowphase
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    iex> a = Body.body( shape: Sphere.sphere(radius: 4), position: {0.0, 0.0, 0.0})
    iex> b = Body.body( shape: Sphere.sphere(radius: 2) , position: {5.0, 0.0, 0.0})
    iex> Narrowphase.test_intersection(a,b)
    {:contact_manifold, {{:contact_point, {3.5, 0.0, 0.0}, 1.0}}, {1.0, 0.0, 0.0}}

    iex> # Check non-touching capsule and sphere
    iex> alias ElixirRigidPhysics.Collision.Narrowphase
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> a = Body.body( shape: Sphere.sphere(radius: 1) , position: {35.0, 0.0, 0.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 1, cap_radius: 0.5), position: {0.0, 0.0, 0.0})
    iex> Narrowphase.test_intersection(a,b)
    :no_intersection

    iex> # Check coincident capsule and sphere
    iex> alias ElixirRigidPhysics.Collision.Narrowphase
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> a = Body.body( shape: Sphere.sphere(radius: 1) , position: {1.0, 0.0, 0.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 2, cap_radius: 0.5), position: {1.0, 0.0, 0.0})
    iex> Narrowphase.test_intersection(a,b)
    :coincident

    iex> # Check side-grazing capsule and sphere
    iex> alias ElixirRigidPhysics.Collision.Narrowphase
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> a = Body.body( shape: Sphere.sphere(radius: 1) , position: {2.0, 0.0, 0.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 2, cap_radius: 1), position: {0.0, 0.0, 0.0})
    iex> Narrowphase.test_intersection(a,b)
    {:contact_manifold, {{:contact_point, {1.0, 0.0, 0.0}, 0.0}}, {-1.0, 0.0, 0.0}}

    iex> # Check top-grazing capsule and sphere
    iex> alias ElixirRigidPhysics.Collision.Narrowphase
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> sqrthalf = :math.sqrt(0.5)
    iex> a = Body.body( shape: Sphere.sphere(radius: 1) , position: {0.0, 0.0, 4.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 4, cap_radius: 1), orientation: {sqrthalf, sqrthalf, 0.0, 0.0})
    iex> {:contact_manifold, {{:contact_point, {0.0, 0.0, 3.0}, distance}}, {0.0, 0.0, -1.0}} = Narrowphase.test_intersection(a,b)
    iex> distance < 0.0001
    true

    iex> # Check partially overlapping sphere and capsule
    iex> # Check side-grazing capsule and sphere
    iex> alias ElixirRigidPhysics.Collision.Narrowphase
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> a = Body.body( shape: Sphere.sphere(radius: 3) , position: {4.0, 0.0, 0.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 12, cap_radius: 2))
    iex> Narrowphase.test_intersection(a,b)
    {:contact_manifold, {{:contact_point, {1.5, 0.0, 0.0}, 1.0}}, {-1.0, 0.0, 0.0}}

    iex> # Check completely contained sphere and capsule
    iex> # Check side-grazing capsule and sphere
    iex> alias ElixirRigidPhysics.Collision.Narrowphase
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> a = Body.body( shape: Sphere.sphere(radius: 2) , position: {3.0, 0.0, 0.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 12, cap_radius: 5))
    iex> Narrowphase.test_intersection(a,b)
    {:contact_manifold, {{:contact_point, {3.0, 0.0, 0.0}, 4.0}}, {-1.0, 0.0, 0.0}}

    iex> # more to do
    iex> "todo"
    "todo"
  """
  @spec test_intersection(Body.body(), Body.body()) :: ContactManifold.contact_manifold()
  def test_intersection(
        Body.body(shape: Sphere.sphere(radius: r_a), position: p_a),
        Body.body(shape: Sphere.sphere(radius: r_b), position: p_b)
      ) do
    a_to_b = Vec3.subtract(p_b, p_a)

    a_to_b_dist_squared = Vec3.length_squared(a_to_b)

    if a_to_b_dist_squared > @verysmol do
      a_to_b_dist = Vec3.length(a_to_b)
      overlap = a_to_b_dist - (r_a + r_b)

      if overlap <= @verysmol do
        penetration_depth = abs(overlap)
        direction = Vec3.normalize(a_to_b)

        ContactManifold.contact_manifold(
          contacts:
            {ContactPoint.contact_point(
               world_point: direction |> Vec3.scale(r_a - penetration_depth / 2) |> Vec3.add(p_a),
               depth: penetration_depth
             )},
          world_normal: direction
        )
      else
        :no_intersection
      end
    else
      # coincident spheres have no sensible contact point or normal, so just give up.
      :coincident
    end
  end

  def test_intersection(
        Body.body(shape: Sphere.sphere(radius: r_a), position: p_a),
        Body.body(shape: Capsule.capsule(cap_radius: cr_b) = c, position: p_b, orientation: o_b)
      ) do
    # note that it might well be faster to transform the sphere to capsule space
    {capsule_local_a, capsule_local_b} = Capsule.get_principle_points(c)
    capsule_a = o_b |> Quatern.transform_vector(capsule_local_a) |> Vec3.add(p_b)
    capsule_b = o_b |> Quatern.transform_vector(capsule_local_b) |> Vec3.add(p_b)

    nearest_in_capsule = GUtil.closest_point_on_line_to_point(p_a, capsule_a, capsule_b)
    vec_to_capsule = Vec3.subtract(nearest_in_capsule, p_a)
    dist_to_capsule = vec_to_capsule |> Vec3.length()

    cond do
      dist_to_capsule > r_a + cr_b ->
        :no_intersection

      dist_to_capsule <= @verysmol ->
        :coincident

      true ->
        overlap = dist_to_capsule - (r_a + cr_b)
        penetration_depth = abs(overlap)
        direction = Vec3.normalize(vec_to_capsule)

        ContactManifold.contact_manifold(
          contacts:
            {ContactPoint.contact_point(
               world_point: direction |> Vec3.scale(r_a - penetration_depth / 2) |> Vec3.add(p_a),
               depth: penetration_depth
             )},
          world_normal: direction
        )
    end
  end
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
