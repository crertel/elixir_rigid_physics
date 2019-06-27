defmodule ElixirRigidPhysics.Util.List do

  @doc """
  Function to find every pair of elements in a list.

  ## Examples

    iex> alias ElixirRigidPhysics.Util.List
    iex> List.find_pairs([])
    []

    iex> alias ElixirRigidPhysics.Util.List
    iex> List.find_pairs([1])
    []

    iex> alias ElixirRigidPhysics.Util.List
    iex> List.find_pairs([1,2])
    [{1,2}]

    iex> alias ElixirRigidPhysics.Util.List
    iex> List.find_pairs([1,2,3])
    [{1,2}, {1,3}, {2,3}]
  """
  @spec find_pairs( [any()] ) :: [ {any(), any()} ]
  def find_pairs([]), do: []
  def find_pairs([_]), do: []
  def find_pairs([a,b]), do: [{a,b}]
  def find_pairs([a | rest]) do
    main_pairs = for i <- rest, do: {a,i}
    main_pairs ++ find_pairs(rest)
  end


  @doc """
  Function to find every pair of elements in a list.

  ## Examples

    iex> alias ElixirRigidPhysics.Util.List
    iex> List.find_pairs([], fn(_a,_b) ->  true end)
    []

    iex> alias ElixirRigidPhysics.Util.List
    iex> List.find_pairs([1], fn(_a,_b) ->  true end)
    []

    iex> alias ElixirRigidPhysics.Util.List
    iex> List.find_pairs([1,2], fn(_a,_b) ->  true end)
    [{1,2}]

    iex> alias ElixirRigidPhysics.Util.List
    iex> List.find_pairs([1,2], fn(_a,_b) -> false end)
    []

    iex> alias ElixirRigidPhysics.Util.List
    iex> List.find_pairs([1,2,3], fn(a,b) -> a * b > 3 end)
    [{2,3}]
  """
  @spec find_pairs( [any()], (any(), any() -> boolean()) ) :: [ {any(), any()} ]
  def find_pairs([], _predicate), do: []
  def find_pairs([_], _predicate), do: []
  def find_pairs([a,b], predicate) do
    if (predicate.(a,b)) do
      [{a,b}]
    else
      []
    end
  end
  def find_pairs([a | rest], predicate) do
    main_pairs = for i <- rest, predicate.(a,i), do: {a,i}
    main_pairs ++ find_pairs(rest)
  end
end
