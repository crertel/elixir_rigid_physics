defmodule ElixirRigidPhysics.Collision.ContactManifold do
  @moduledoc """
  Module for contact manifolds.

  Contact manifolds are used during the narrowphase collision detection. They give the normal of collisin as well as up to four contact points.
  """
  alias Graphmath.Vec3
  alias ElixirRigidPhysics.Collision.ContactPoint

  require Record

  Record.defrecord(:contact_manifold,
    contacts: {},
    world_normal: {0.0, 0.0, 0.0}
  )

  @type contact_manifold ::
          record(:contact_manifold,
            contacts:
              {}
              | {ContactPoint.contact_point()}
              | {ContactPoint.contact_point(), ContactPoint.contact_point()}
              | {ContactPoint.contact_point(), ContactPoint.contact_point(),
                 ContactPoint.contact_point()}
              | {ContactPoint.contact_point(), ContactPoint.contact_point(),
                 ContactPoint.contact_point(), ContactPoint.contact_point()},
            world_normal: Vec3.vec3()
          )
end
