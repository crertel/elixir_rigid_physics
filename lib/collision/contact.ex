defmodule ElixirRigidPhysics.Collision.Contact do
  @moduledoc """
  Module for contact manifolds, points, and results.

  Contact manifolds are used during the narrowphase collision detection. They give the normal of collisin as well as up to four contact points.
  """
  alias Graphmath.Vec3

  require Record

  Record.defrecord(:contact_manifold,
    contacts: {},
    world_normal: {0.0, 0.0, 0.0}
  )

  Record.defrecord(:contact_point, world_point: {0.0, 0.0, 0.0}, depth: 0.0)
  @type contact_point :: record(:contact_point, world_point: Vec3.vec3, depth: number)

  @type contact_manifold ::
          record(:contact_manifold,
            contacts:
              {}
              | {contact_point()}
              | {contact_point(), contact_point()}
              | {contact_point(), contact_point(),
                 contact_point()}
              | {contact_point(), contact_point(),
                 contact_point(), contact_point()},
            world_normal: Vec3.vec3
          )

  @type contact_result :: contact_manifold() | :no_intersection | :coincident | {:error, :bad_bodies}
end
