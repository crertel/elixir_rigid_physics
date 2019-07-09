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
    {:contact_manifold, {{:contact_point, {0.0, 2.0, 0.0}, 14.0}}, {0.0, 1.0, 0.0}}

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
  """
  def check(
        Body.body(shape: Capsule.capsule(cap_radius: cr_a) = cap_a, position: p_a, orientation: o_a),
        Body.body(shape: Capsule.capsule(cap_radius: cr_b) = cap_b, position: p_b, orientation: o_b)
      ) do
      {capsule_local_a_1, capsule_local_a_2} = Capsule.get_principle_points(cap_a)
      {capsule_local_b_1, capsule_local_b_2} = Capsule.get_principle_points(cap_b)
      capsule_world_a_1 = o_a |> Quatern.transform_vector(capsule_local_a_1) |> Vec3.add(p_a)
      capsule_world_a_2 = o_a |> Quatern.transform_vector(capsule_local_a_2) |> Vec3.add(p_a)
      capsule_world_b_1 = o_b |> Quatern.transform_vector(capsule_local_b_1) |> Vec3.add(p_b)
      capsule_world_b_2 = o_b |> Quatern.transform_vector(capsule_local_b_2) |> Vec3.add(p_b)

      {dist, cap_a_nearest, cap_b_nearest} = GUtil.nearest_points_for_segments({capsule_world_a_1, capsule_world_a_2}, {capsule_world_b_1, capsule_world_b_2})

      min_distance = cr_a + cr_b

      cond do
        dist > min_distance -> :no_intersection
        dist < @verysmol -> :coincident
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
                  world_point: direction |> Vec3.scale(cr_a - penetration_depth / 2) |> Vec3.add(p_a),
                  depth: penetration_depth
                )},
              world_normal: direction
            )
          else
            # happy capsules :)
            a_axis = Vec3.scale(a_spine, 1.0/ :math.sqrt(a_spine_lengthsq))
            b_axis = Vec3.scale(b_spine, 1.0/ :math.sqrt(b_spine_lengthsq))
            capsule_angle = :math.acos( Vec3.dot(a_axis, b_axis))
            if :math.fmod( capsule_angle, :math.pi) > @verysmol do
              # non-parallel capsules, 1 contact poiont
              a_to_b = Vec3.subtract(cap_b_nearest, cap_a_nearest)
              a_to_b_dist = Vec3.length(a_to_b)
              overlap = a_to_b_dist - (min_distance)
              penetration_depth = abs(overlap)
              direction = Vec3.normalize(a_to_b)

              ContactManifold.contact_manifold(
                contacts:
                  {ContactPoint.contact_point(
                    world_point: direction |> Vec3.scale(cr_a - penetration_depth / 2) |> Vec3.add(cap_a_nearest),
                    depth: penetration_depth
                  )},
                world_normal: direction
              )
            else
              # parallel (stacked!) capsules, 2 contact points

              # clip spine of secondary shape to primary shape (note a-axis points inwards)
              a_21_axis = a_axis
              a_12_axis = Vec3.scale(a_axis, -1.0)
              a_plane_1 = Plane.create(a_12_axis, capsule_world_a_1)
              a_plane_2 = Plane.create(a_21_axis, capsule_world_a_2)

              half_clipped_b_1 = Plane.clip_point(a_plane_1, capsule_world_b_1)
              half_clipped_b_2 = Plane.clip_point(a_plane_2, capsule_world_b_2)
              clipped_b_1 = Plane.clip_point(a_plane_1, half_clipped_b_1)
              clipped_b_2 = Plane.clip_point(a_plane_2, half_clipped_b_2)

              clipped_nearest_1 = GUtil.closest_point_on_line_to_point(clipped_b_1, capsule_world_a_1, capsule_world_a_2)
              clipped_nearest_2 = GUtil.closest_point_on_line_to_point(clipped_b_2, capsule_world_a_1, capsule_world_a_2)
              dir_clipped_1 = Vec3.subtract(clipped_nearest_1, clipped_b_1)
              dir_clipped_2 = Vec3.subtract(clipped_nearest_2, clipped_b_2)
              distance_clipped_1 = dir_clipped_1 |> Vec3.length()
              distance_clipped_2 = dir_clipped_2 |> Vec3.length()

              cond do
                distance_clipped_1 <= min_distance and distance_clipped_2 <= min_distance ->
                  overlap1 = distance_clipped_1 - (min_distance)
                  overlap2 = distance_clipped_2 - (min_distance)
                  penetration_depth1 = abs(overlap1)
                  penetration_depth2 = abs(overlap2)
                  ndir_clipped_1 = Vec3.normalize(dir_clipped_1)
                  ndir_clipped_2 = Vec3.normalize(dir_clipped_2)

                  ContactManifold.contact_manifold(
                  contacts:
                    {ContactPoint.contact_point(
                      world_point: ndir_clipped_1 |> Vec3.scale(cr_a - penetration_depth1 / 2) |> Vec3.add(clipped_nearest_1),
                      depth: penetration_depth1
                    ),
                    ContactPoint.contact_point(
                      world_point: ndir_clipped_2 |> Vec3.scale(cr_a - penetration_depth2 / 2) |> Vec3.add(clipped_nearest_2),
                      depth: penetration_depth2
                    )},
                  world_normal: ndir_clipped_1
                )

                distance_clipped_2 <= min_distance -> # 1 is out of distance, somehow
                  overlap2 = distance_clipped_2 - (min_distance)
                  penetration_depth2 = abs(overlap2)
                  ndir_clipped_2 = Vec3.normalize(dir_clipped_2)
                  ContactManifold.contact_manifold(
                  contacts:
                    {ContactPoint.contact_point(
                      world_point: ndir_clipped_2 |> Vec3.scale(cr_a - penetration_depth2 / 2) |> Vec3.add(clipped_nearest_2),
                      depth: penetration_depth2
                    )},
                  world_normal: ndir_clipped_2
                )

                distance_clipped_1 <= min_distance -> # 2 is out of distance, somehow
                  overlap1 = distance_clipped_1 - (min_distance)
                  penetration_depth1 = abs(overlap1)
                  IO.inspect(dir_clipped_1, label: "ah")
                  ndir_clipped_1 = Vec3.normalize(dir_clipped_1)
                  ContactManifold.contact_manifold(
                  contacts:
                    {ContactPoint.contact_point(
                      world_point: ndir_clipped_1 |> Vec3.scale(cr_a - penetration_depth1 / 2) |> Vec3.add(clipped_nearest_1),
                      depth: penetration_depth1
                    )},
                  world_normal: ndir_clipped_1
                )
              end
            end
          end
      end
  end
end
