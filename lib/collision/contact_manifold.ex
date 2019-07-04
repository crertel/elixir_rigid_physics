defmodule ElixirRigidPhysics.Collision.ContactManifold do
  require Record

  Record.defrecord(:contact_manifold,
    contacts: {},
    world_normal: {0.0, 0.0, 0.0}
  )
end
