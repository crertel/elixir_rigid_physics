defmodule ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule do
  @moduledoc """
  Module for sphere-capsule intersection tests.
  """

  require ElixirRigidPhysics.Dynamics.Body, as: Body
  require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
  require ElixirRigidPhysics.Collision.ContactManifold, as: ContactManifold
  require ElixirRigidPhysics.Collision.ContactPoint, as: ContactPoint

  alias ElixirRigidPhysics.Geometry.Util, as: GUtil
  alias ElixirRigidPhysics.Geometry.Plane, as: Plane
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
    iex> CapsuleCapsule.check(a,b)
    :no_intersection

    iex> # Check coincident capsules
    iex> alias ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> a = Body.body( shape: Capsule.capsule(axial_length: 1, cap_radius: 0.5), position: {0.0, 0.0, 0.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 1, cap_radius: 0.5), position: {0.0, 0.0, 0.0})
    iex> CapsuleCapsule.check(a,b)
    :coincident

    iex> # Check degenerate capsule contact
    iex> alias ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> a = Body.body( shape: Capsule.capsule(axial_length: 0.0, cap_radius: 1.0), position: {0.0, 0.0, 0.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 2.0, cap_radius: 1.0), position: {0.0, 3.0, 0.0})
    iex> CapsuleCapsule.check(a,b)
    {:contact_manifold, {{:contact_point, {0.0, 1.0, 0.0}, 0.0}}, {0.0, 1.0, 0.0}}

    iex> # Check grazing cap contact
    iex> alias ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> a = Body.body( shape: Capsule.capsule(axial_length: 3.0, cap_radius: 0.5), position: {0.0, 0.0, 0.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 2.0, cap_radius: 1.0), position: {0.0, 4.0, 0.0})
    iex> CapsuleCapsule.check(a,b)
    {:contact_manifold, {{:contact_point, {0.0, 2.0, 0.0}, 0.0}}, {0.0, 1.0, 0.0}}

    iex> # Check penetrating grazing cap contact
    iex> alias ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> a = Body.body( shape: Capsule.capsule(axial_length: 3.0, cap_radius: 0.5), position: {0.0, 0.0, 0.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 2.0, cap_radius: 1.0), position: {0.0, 3.75, 0.0})
    iex> CapsuleCapsule.check(a,b)
    {:contact_manifold, {{:contact_point, {0.0, 1.875, 0.0}, 0.25}}, {0.0, 1.0, 0.0}}

    iex> # Check penetrating side contact
    iex> alias ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> sqrthalf = :math.sqrt(0.5)
    iex> a = Body.body( shape: Capsule.capsule(axial_length: 2.0, cap_radius: 1), position: {0.0, 0.0, 3.5})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 4.0, cap_radius: 1), orientation: {sqrthalf, sqrthalf, 0.0, 0.0})
    iex> res = CapsuleCapsule.check(a,b)
    iex> match?({:contact_manifold, {{:contact_point, {0.0, 0.0, 2.75}, _}}, {0.0, 0.0, -1.0}} ,res )
    true
    iex> {:contact_manifold, {{:contact_point, {0.0, 0.0, 2.75}, d}}, {0.0, 0.0, -1.0}} = res
    iex> Float.round(d,2) == 0.5
    true

    iex> # IO.puts("Check stacking")
    iex> alias ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> a = Body.body( shape: Capsule.capsule(axial_length: 2.0, cap_radius: 1.0), position: {2.0, 0.0, 0.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 2.0, cap_radius: 1.0), position: {0.0, 0.0, 0.0})
    iex> CapsuleCapsule.check(a,b)
    {:contact_manifold, {{:contact_point, {1.0, -1.0, 0.0}, 0.0}, {:contact_point, {1.0, 1.0, 0.0}, 0.0}}, {-1.0, 0.0, 0.0}}

    iex> # IO.puts("Check stacking with contained, smaller b")
    iex> alias ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> a = Body.body( shape: Capsule.capsule(axial_length: 2.0, cap_radius: 1.0), position: {2.0, 0.0, 0.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 1.0, cap_radius: 1.0), position: {0.0, 0.0, 0.0})
    iex> CapsuleCapsule.check(a,b)
    {:contact_manifold, {{:contact_point, {1.0, -0.5, 0.0}, 0.0}, {:contact_point, {1.0, 0.5, 0.0}, 0.0}}, {-1.0, 0.0, 0.0}}

    iex> # IO.puts("Check stacking with uncontained, larger b")
    iex> alias ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> a = Body.body( shape: Capsule.capsule(axial_length: 2.0, cap_radius: 1.0), position: {2.0, 0.0, 0.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 20.0, cap_radius: 1.0), position: {0.0, 0.0, 0.0})
    iex> CapsuleCapsule.check(a,b)
    {:contact_manifold, {{:contact_point, {1.0, -1.0, 0.0}, 0.0}, {:contact_point, {1.0, 1.0, 0.0}, 0.0}}, {-1.0, 0.0, 0.0}}

    iex> # IO.puts("Check stacking of transformed capsules")
    iex> alias ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> sqrthalf = :math.sqrt(0.5)
    iex> a = Body.body( shape: Capsule.capsule(axial_length: 2.0, cap_radius: 1.0), position: {0.0, 2.0, 0.0}, orientation: {sqrthalf, sqrthalf, 0.0, 0.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 4.0, cap_radius: 1.0), position: {0.0, 0.0, 0.0}, orientation: {sqrthalf, sqrthalf, 0.0, 0.0})
    iex> {:contact_manifold, {{:contact_point, cp1, cp1d}, {:contact_point, cp2, cp2d}}, cmn} = CapsuleCapsule.check(a,b)
    iex> Float.round(cp1d, 3) == 0.00
    true
    iex> Float.round(cp2d, 3) == 0.00
    true
    iex> Graphmath.Vec3.equal(cp1, {0.0, 1.0, -1.0}, 1.0e-6)
    true
    iex> Graphmath.Vec3.equal(cp2, {0.0, 1.0, 1.0}, 1.0e-6)
    true
    iex> Graphmath.Vec3.equal(cmn, {0.0, -1.0, 0.0}, 1.0e-6)
    true

    iex> # IO.puts("Check deeper stacking")
    iex> alias ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> a = Body.body( shape: Capsule.capsule(axial_length: 2.0, cap_radius: 1.0), position: {1.5, 0.0, 0.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 2.0, cap_radius: 1.0), position: {0.0, 0.0, 0.0})
    iex> CapsuleCapsule.check(a,b)
    {:contact_manifold, {{:contact_point, {0.75, -1.0, 0.0}, 0.5}, {:contact_point, {0.75, 1.0, 0.0}, 0.5}}, {-1.0, 0.0, 0.0}}
  """
  def check(
        Body.body(
          shape: Capsule.capsule(cap_radius: cr_a) = cap_a,
          position: p_a,
          orientation: o_a
        ),
        Body.body(
          shape: Capsule.capsule(cap_radius: cr_b) = cap_b,
          position: p_b,
          orientation: o_b
        )
      ) do
    {capsule_local_a_1, capsule_local_a_2} = Capsule.get_principle_points(cap_a)
    {capsule_local_b_1, capsule_local_b_2} = Capsule.get_principle_points(cap_b)
    capsule_world_a_1 = o_a |> Quatern.transform_vector(capsule_local_a_1) |> Vec3.add(p_a)
    capsule_world_a_2 = o_a |> Quatern.transform_vector(capsule_local_a_2) |> Vec3.add(p_a)
    capsule_world_b_1 = o_b |> Quatern.transform_vector(capsule_local_b_1) |> Vec3.add(p_b)
    capsule_world_b_2 = o_b |> Quatern.transform_vector(capsule_local_b_2) |> Vec3.add(p_b)

    {dist, cap_a_nearest, cap_b_nearest} =
      GUtil.nearest_points_for_segments(
        {capsule_world_a_1, capsule_world_a_2},
        {capsule_world_b_1, capsule_world_b_2}
      )

    min_distance = cr_a + cr_b

    cond do
      dist > min_distance ->
        :no_intersection

      dist < @verysmol ->
        :coincident

      true ->
        # we need to see if the capsules are parallel or not,
        a_spine = Vec3.subtract(capsule_world_a_2, capsule_world_a_1)
        b_spine = Vec3.subtract(capsule_world_b_2, capsule_world_b_1)
        a_spine_lengthsq = Vec3.length_squared(a_spine)
        b_spine_lengthsq = Vec3.length_squared(b_spine)

        if a_spine_lengthsq < @verysmol or b_spine_lengthsq < @verysmol do
          # degenerate capsules :(, 1 contact point
          a_to_b = Vec3.subtract(cap_b_nearest, cap_a_nearest)
          a_to_b_dist = Vec3.length(a_to_b)
          overlap = a_to_b_dist - (cr_a + cr_b)
          penetration_depth = abs(overlap)
          direction = Vec3.normalize(a_to_b)

          ContactManifold.contact_manifold(
            contacts:
              {ContactPoint.contact_point(
                 world_point:
                   direction |> Vec3.scale(cr_a - penetration_depth / 2) |> Vec3.add(p_a),
                 depth: penetration_depth
               )},
            world_normal: direction
          )
        else
          # happy capsules :)
          a_axis = Vec3.scale(a_spine, 1.0 / :math.sqrt(a_spine_lengthsq))
          b_axis = Vec3.scale(b_spine, 1.0 / :math.sqrt(b_spine_lengthsq))
          capsule_angle = :math.acos(Vec3.dot(a_axis, b_axis))
          a_to_b = Vec3.subtract(cap_b_nearest, cap_a_nearest)
          a_to_b_dist = Vec3.length(a_to_b)
          direction = Vec3.normalize(a_to_b)

          if :math.fmod(capsule_angle, :math.pi()) > @verysmol do
            # non-parallel capsules, 1 contact poiont
            overlap = a_to_b_dist - min_distance
            penetration_depth = abs(overlap)

            ContactManifold.contact_manifold(
              contacts:
                {ContactPoint.contact_point(
                   world_point:
                     direction
                     |> Vec3.scale(cr_a - penetration_depth / 2)
                     |> Vec3.add(cap_a_nearest),
                   depth: penetration_depth
                 )},
              world_normal: direction
            )
          else
            # capsules spines are parallel!
            # HOWEVER, we don't know if the capsules are butting up against each other,
            # if the second capsule is entirely "within" the first capsule's axis,
            # or if there is a partial overlap.
            # The latter two cases will require 2 contact points, the former only one.

            # get the planes for the "interior" of capsule 1
            a_21_axis = Vec3.scale(a_axis, -1.0)
            a_12_axis = a_axis
            a_plane_1 = Plane.create(a_12_axis, capsule_world_a_1)
            a_plane_2 = Plane.create(a_21_axis, capsule_world_a_2)

            # we're gonna check to see if the capsule spines are entirely disjoint
            # if both points are in front of one plane and behind another, capsule b is abutted
            cwb1_to_ap1_dist = Plane.distance_to_point(a_plane_1, capsule_world_b_1)
            cwb2_to_ap1_dist = Plane.distance_to_point(a_plane_1, capsule_world_b_2)
            cwb1_to_ap2_dist = Plane.distance_to_point(a_plane_2, capsule_world_b_1)
            cwb2_to_ap2_dist = Plane.distance_to_point(a_plane_2, capsule_world_b_2)

            if (cwb1_to_ap1_dist < 0 and cwb2_to_ap1_dist < 0) or
                 (cwb1_to_ap2_dist < 0 and cwb2_to_ap2_dist < 0) do
              # capsule b is abutting capsule a
              overlap = a_to_b_dist - min_distance
              penetration_depth = abs(overlap)

              ContactManifold.contact_manifold(
                contacts:
                  {ContactPoint.contact_point(
                     world_point:
                       direction
                       |> Vec3.scale(cr_a - penetration_depth / 2)
                       |> Vec3.add(cap_a_nearest),
                     depth: penetration_depth
                   )},
                world_normal: direction
              )
            else
              # clip spine of secondary shape to primary shape (note a-axis points inwards)
              # this works because the spines are parallel, and the closest point on line is on the plane normal to the spine
              clipped_nearest_1 =
                GUtil.closest_point_on_line_to_point(
                  capsule_world_a_1,
                  capsule_world_b_1,
                  capsule_world_b_2
                )

              clipped_nearest_2 =
                GUtil.closest_point_on_line_to_point(
                  capsule_world_a_2,
                  capsule_world_b_1,
                  capsule_world_b_2
                )

              distance_clipped = a_to_b_dist
              overlap = distance_clipped - min_distance
              penetration_depth = abs(overlap)

              ContactManifold.contact_manifold(
                contacts:
                  {ContactPoint.contact_point(
                     world_point:
                       direction
                       |> Vec3.scale(-(cr_a - penetration_depth / 2))
                       |> Vec3.add(clipped_nearest_1),
                     depth: penetration_depth
                   ),
                   ContactPoint.contact_point(
                     world_point:
                       direction
                       |> Vec3.scale(-(cr_a - penetration_depth / 2))
                       |> Vec3.add(clipped_nearest_2),
                     depth: penetration_depth
                   )},
                world_normal: direction
              )
            end
          end
        end
    end
  end
end
