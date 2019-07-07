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

    iex> # Check grazing cap contact
    iex> alias ElixirRigidPhysics.Collision.Intersection.CapsuleCapsule
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> a = Body.body( shape: Capsule.capsule(axial_length: 3.0, cap_radius: 0.5), position: {0.0, 0.0, 0.0})
    iex> b = Body.body( shape: Capsule.capsule(axial_length: 2.0, cap_radius: 1.0), position: {0.0, 4.0, 0.0})
    iex> CapsuleCapsule.check(a,b)
    {:contact_manifold, {{:contact_point, {0.0, 2.0, 0.0}, 1.0}}, {0.0, 1.0, 0.0}}

  """
  def check(
        Body.body(shape: Capsule.capsule(cap_radius: cr_a) = cap_a, position: p_a, orientation: o_b),
        Body.body(shape: Capsule.capsule(cap_radius: cr_b) = cap_b, position: p_b, orientation: o_b)
      ) do
      {capsule_local_a_1, capsule_local_a_2} = Capsule.get_principle_points(cap_a)
      {capsule_local_b_1, capsule_local_b_2} = Capsule.get_principle_points(cap_b)
      capsule_world_a_1 = o_b |> Quatern.transform_vector(capsule_local_a_1) |> Vec3.add(p_a)
      capsule_world_a_2 = o_b |> Quatern.transform_vector(capsule_local_a_2) |> Vec3.add(p_a)
      capsule_world_b_1 = o_b |> Quatern.transform_vector(capsule_local_b_1) |> Vec3.add(p_b)
      capsule_world_b_2 = o_b |> Quatern.transform_vector(capsule_local_b_2) |> Vec3.add(p_b)

      {dist, cap_a_nearest, cap_b_nearest} = GUtil.nearest_points_for_segments({capsule_world_a_1, capsule_world_a_2}, {capsule_world_b_1, capsule_world_b_2})

      cond do
        dist > cr_a + cr_b -> :no_intersection
        dist < @verysmol -> :coincident
        true ->
          # we need to see if the capsules are parallel or not,
          a_spine = Vec3.subtract(capsule_world_a_2, capsule_world_a_1)
          b_spine = Vec3.subtract(capsule_world_b_2, capsule_world_b_1)
          a_spine_lengthsq = Vec3.length_squared(a_spine)
          b_spine_lengthsq = Vec3.length_squared(b_spine)
          if a_spine_lengthsq < @verysmol or b_spine_lengthsq < @verysmol do
            # degenerate capsules :(, 1 contact point
          else
            # happy caqpsules :)
            a_axis = Vec3.scale(a_spine, :math.sqrt(a_spine_lengthsq))
            b_axis = Vec3.scale(b_spine, :math.sqrt(b_spine_lengthsq))
            capsule_angle = :math.acos( Vec3.dot(a_axis, b_axis))
            if :math.fmod( capsule_angle, :math.pi) > @verysmol do
              # non-parallel capsules, 1 contact poiont
            else
              # parallel (stacked!) capsules, 2 contact points
            end
          end
      end
  end
end
