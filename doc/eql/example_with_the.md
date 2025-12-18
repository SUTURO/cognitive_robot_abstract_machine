# Example with `the`

`the` is an entity query wrapper like the `an`. But, unlike the `an` which allows for multiple solutions, `the` only
allows one possible solution. If `the` is used and more than one solution was found, an error will be raised.

This is similar to `one` in SQL.

## Example Usage

```python
from dataclasses import dataclass

from typing_extensions import List

from entity_query_language import entity, let, the, MultipleSolutionFound, symbol, symbolic_mode


@symbol
@dataclass
class Body:
    name: str

@symbol
@dataclass
class World:
    id_: int
    bodies: List[Body]


world = World(1, [Body("Body1"), Body("Body2")])

with symbolic_mode():
    body1 = the(entity(body := let(type_=Body, domain=world.bodies),
                       body.name.startswith("Body1"))).evaluate()
    try:
        body = the(entity(body := let(type_=Body, domain=world.bodies),
                          body.name.startswith("Body"))).evaluate()
        assert False
    except MultipleSolutionFound:
        pass
```

`body1` will execute successfully giving one solution wich is the body with the name `Body1`.
`body` will raise an error is there is multiple bodies which have a name that starts with `Body`.
