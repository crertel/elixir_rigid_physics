defmodule ElixirRigidPhysics.Collision.AABB do
  require Record
  Record.defrecord(:aabb, min: {0.0, 0.0, 0.0}, max: {0.0, 0.0, 0.0})

  require ElixirRigidPhysics.Dynamics.Body, as: Body
  require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
  require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
  require ElixirRigidPhysics.Geometry.Box, as: Box

  def create_world_from_body(
        Body.body(shape: Sphere.sphere(radius: r) = _sphere, position: {px, py, pz})
      ) do
    aabb(
      min: {px - r, py - r, pz - r},
      max: {px + r, py + r, pz + r}
    )
  end

  def create_world_from_body(
        Body.body(shape: shape, position: {px, py, pz}, orientation: orientation)
      ) do
    # get local-space AABB
    _aabb = create_local_from_shape(shape)

    # rotate to match world frame

    # calculate new AABB

    # offset by position
  end

  @doc """
  Creates an AABB from a Box geometry, in local space centered on origin.

  Width is the length on the x-axis, height is the length on the y-axis, and depth is the length on the z-axis.

  ## Examples


    iex> require ElixirRigidPhysics.Collision.AABB, as: AABB
    ElixirRigidPhysics.Collision.AABB
    iex> require ElixirRigidPhysics.Geometry.Box, as: Box
    ElixirRigidPhysics.Geometry.Box
    iex> AABB.create_local_from_shape( Box.box(width: 1.0, height: 2.0, depth: 3.0))
    {:aabb, {-0.5, -1.0, -1.5}, {0.5, 1.0, 1.5}}

    iex> require ElixirRigidPhysics.Collision.AABB, as: AABB
    ElixirRigidPhysics.Collision.AABB
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    ElixirRigidPhysics.Geometry.Sphere
    iex> AABB.create_local_from_shape( Sphere.sphere(radius: 1.0))
    {:aabb, {-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}}
    iex> AABB.create_local_from_shape( Sphere.sphere(radius: 2.0))
    {:aabb, {-2.0, -2.0, -2.0}, {2.0, 2.0, 2.0}}

    iex> require ElixirRigidPhysics.Collision.AABB, as: AABB
    ElixirRigidPhysics.Collision.AABB
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    ElixirRigidPhysics.Geometry.Capsule
    iex> AABB.create_local_from_shape( Capsule.capsule(axial_length: 1.0, cap_radius: 0.5))
    {:aabb, {-0.5, -1.0, -0.5}, {0.5, 1.0, 0.5}}

  """
  def create_local_from_shape(Box.box(width: w, height: h, depth: d)) do
    aabb(
      min: {-w / 2, -h / 2, -d / 2},
      max: {w / 2, h / 2, d / 2}
    )
  end

  def create_local_from_shape(Sphere.sphere(radius: r)) do
    aabb(
      min: {-r, -r, -r},
      max: {r, r, r}
    )
  end

  def create_local_from_shape(Capsule.capsule(axial_length: al, cap_radius: cr)) do
    half_height = cr + al / 2.0

    aabb(
      min: {-cr, -half_height, -cr},
      max: {cr, half_height, cr}
    )
  end
end
