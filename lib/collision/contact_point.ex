defmodule ElixirRigidPhysics.Collision.ContactPoint do
  require Record

  Record.defrecord(:contact_point,
                    world_point: {0.0, 0.0, 0.0}, depth: 0.0)
end
