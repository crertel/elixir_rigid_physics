defmodule ElixirRigidPhysics.Bodies.Box do
  alias Graphmath.Mat44

  def make(l, w, d, {tx, ty, tz} = _pos, {_rx,_ry,_rz,_rw} = rotquat, {_mx, _my, _mz} = moi, mass)
      when is_number(l) and l > 0 and is_number(w) and w > 0 and is_number(d) and d > 0 do
    xform = Mat44.make_translate(tx, ty, tz)

    %{
      type: :box,
      length: l,
      width: w,
      depth: d,

      rotation: rotquat,
      mass: mass,
      moi: moi,

      transform: xform
    }
  end

  def make(l, w, d)
      when is_number(l) and l > 0 and is_number(w) and w > 0 and is_number(d) and d > 0,
      do: make(l, w, d, {0.0, 0.0, 0.0}, {0.0,0.0,0.0,0.0}, {0.0,0.0,0.0}, 1)
end
