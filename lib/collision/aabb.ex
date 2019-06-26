defmodule ElixirRigidPhysics.Collision.AABB do
  require Record
  Record.defrecord(:aabb, min: {0.0, 0.0, 0.0}, max: {0.0, 0.0, 0.0})

  require ElixirRigidPhysics.Dynamics.Body, as: Body
  require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
  require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
  require ElixirRigidPhysics.Geometry.Box, as: Box

  def create_world_from_body( Body.body(shape: Sphere.sphere(radius: r) = _sphere, position: {px, py, pz})) do
    aabb(
      min: { px - r, py - r, pz - r },
      max: { px + r, py + r, pz + r}
    )
  end

  def create_world_from_body( Body.body(shape: shape, position: {px, py, pz}, orientation: orientation)) do

    # get local-space AABB
    _aabb = create_local_from_shape(shape)

    # rotate to match world frame

    # calculate new AABB

    # offset by position
  end

  def create_local_from_shape( Capsule.capsule(axial_length: al, cap_radius: cr) ) do

    height = (2.0 * cr) + al

    aabb(
      min: { -cr, -cr, -height },
      max: { cr, cr, height}
    )
  end

  def create_local_from_shape( Box.box(length: l, width: w, depth: d) ) do
    aabb(
      min: { -l/2, -w/2, -d/2 },
      max: { l/2, w/2, d/2}
    )
  end

  def create_local_from_shape( Sphere.sphere(radius: r)) do
    aabb(
      min: { -r, -r, -r },
      max: { r, r, r}
    )
  end
end
