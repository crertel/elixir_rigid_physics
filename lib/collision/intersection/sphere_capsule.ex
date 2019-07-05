defmodule ElixirRigidPhysics.Collision.Intersection.SphereCapsule do

    @moduledoc """
    Module for sphere-capsule intersection tests.
    """

    require ElixirRigidPhysics.Dynamics.Body, as: Body
    require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    require ElixirRigidPhysics.Collision.ContactManifold, as: ContactManifold
    require ElixirRigidPhysics.Collision.ContactPoint, as: ContactPoint

    alias ElixirRigidPhysics.Geometry.Util, as: GUtil
    alias Graphmath.Vec3
    alias Graphmath.Quatern

    @verysmol 1.0e-12

    @doc """
    Check the intersection of a sphere and a capsule.

    ## Examples
      iex> # Check non-touching capsule and sphere
      iex> alias ElixirRigidPhysics.Collision.Intersection.SphereCapsule
      iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
      iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
      iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
      iex> a = Body.body( shape: Sphere.sphere(radius: 1) , position: {35.0, 0.0, 0.0})
      iex> b = Body.body( shape: Capsule.capsule(axial_length: 1, cap_radius: 0.5), position: {0.0, 0.0, 0.0})
      iex> SphereCapsule.check(a,b)
      :no_intersection

      iex> # Check coincident capsule and sphere
      iex> alias ElixirRigidPhysics.Collision.Intersection.SphereCapsule
      iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
      iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
      iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
      iex> a = Body.body( shape: Sphere.sphere(radius: 1) , position: {1.0, 0.0, 0.0})
      iex> b = Body.body( shape: Capsule.capsule(axial_length: 2, cap_radius: 0.5), position: {1.0, 0.0, 0.0})
      iex> SphereCapsule.check(a,b)
      :coincident

      iex> # Check side-grazing capsule and sphere
      iex> alias ElixirRigidPhysics.Collision.Intersection.SphereCapsule
      iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
      iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
      iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
      iex> a = Body.body( shape: Sphere.sphere(radius: 1) , position: {2.0, 0.0, 0.0})
      iex> b = Body.body( shape: Capsule.capsule(axial_length: 2, cap_radius: 1), position: {0.0, 0.0, 0.0})
      iex> SphereCapsule.check(a,b)
      {:contact_manifold, {{:contact_point, {1.0, 0.0, 0.0}, 0.0}}, {-1.0, 0.0, 0.0}}

      iex> # Check top-grazing capsule and sphere
      iex> alias ElixirRigidPhysics.Collision.Intersection.SphereCapsule
      iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
      iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
      iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
      iex> sqrthalf = :math.sqrt(0.5)
      iex> a = Body.body( shape: Sphere.sphere(radius: 1) , position: {0.0, 0.0, 4.0})
      iex> b = Body.body( shape: Capsule.capsule(axial_length: 4, cap_radius: 1), orientation: {sqrthalf, sqrthalf, 0.0, 0.0})
      iex> {:contact_manifold, {{:contact_point, {0.0, 0.0, 3.0}, distance}}, {0.0, 0.0, -1.0}} = SphereCapsule.check(a,b)
      iex> distance < 0.0001
      true

      iex> # Check partially overlapping sphere and capsule
      iex> # Check side-grazing capsule and sphere
      iex> alias ElixirRigidPhysics.Collision.Intersection.SphereCapsule
      iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
      iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
      iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
      iex> a = Body.body( shape: Sphere.sphere(radius: 3) , position: {4.0, 0.0, 0.0})
      iex> b = Body.body( shape: Capsule.capsule(axial_length: 12, cap_radius: 2))
      iex> SphereCapsule.check(a,b)
      {:contact_manifold, {{:contact_point, {1.5, 0.0, 0.0}, 1.0}}, {-1.0, 0.0, 0.0}}

      iex> # Check completely contained sphere and capsule
      iex> # Check side-grazing capsule and sphere
      iex> alias ElixirRigidPhysics.Collision.Intersection.SphereCapsule
      iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
      iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
      iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
      iex> a = Body.body( shape: Sphere.sphere(radius: 2) , position: {3.0, 0.0, 0.0})
      iex> b = Body.body( shape: Capsule.capsule(axial_length: 12, cap_radius: 5))
      iex> SphereCapsule.check(a,b)
      {:contact_manifold, {{:contact_point, {3.0, 0.0, 0.0}, 4.0}}, {-1.0, 0.0, 0.0}}
    """
    def check(
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
