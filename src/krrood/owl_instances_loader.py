from __future__ import annotations

from dataclasses import fields, is_dataclass
from types import ModuleType
from typing import Any, Dict, Iterable, List, Optional, Tuple, Type, Union

import rdflib
from rdflib import RDF, RDFS, URIRef, Literal


class OwlInstancesRegistry:
    """Registry of instances created from an OWL/RDF instances file.

    Provides access to instances per Python model class and tracks URIRef to instance mapping.
    """

    def __init__(self) -> None:
        self._by_uri: Dict[URIRef, Any] = {}
        self._by_class: Dict[Type, List[Any]] = {}

    def get_or_create_for(self, uri: URIRef, factory: Type) -> Any:
        inst = self._by_uri.get(uri)
        if inst is None:
            inst = factory()
            # Fill a best-effort human-readable name if available
            local = local_name(uri)
            if hasattr(inst, "name") and getattr(inst, "name") is None:
                setattr(inst, "name", local)
            self._by_uri[uri] = inst
            self._by_class.setdefault(factory, []).append(inst)
        return inst

    def get(self, cls: Type) -> List[Any]:
        return list(self._by_class.get(cls, []))

    def resolve(self, uri: URIRef) -> Optional[Any]:
        return self._by_uri.get(uri)


def local_name(uri: Union[str, URIRef]) -> str:
    s = str(uri)
    if "#" in s:
        return s.rsplit("#", 1)[1]
    return s.rstrip("/").rsplit("/", 1)[-1]


def to_snake(name: str) -> str:
    out = []
    for i, ch in enumerate(name):
        if ch.isupper() and i > 0 and (not name[i - 1].isupper()):
            out.append("_")
        out.append(ch.lower())
    return "".join(out)


def to_pascal(name: str) -> str:
    parts = []
    cur = []
    for ch in name:
        if ch == "_":
            if cur:
                parts.append("".join(cur))
                cur = []
        else:
            cur.append(ch)
    if cur:
        parts.append("".join(cur))
    return "".join(p.capitalize() for p in parts)


def _collect_model_metadata(model_module) -> Tuple[
    Dict[str, Type],  # class name -> class type
    Dict[str, Type],  # descriptor base name -> descriptor class
    Dict[Type, Dict[str, str]],  # class -> predicate(local) -> field name
    Dict[Type, Dict[Type, str]],  # class -> descriptor class -> field name
]:
    class_by_name: Dict[str, Type] = {}
    descriptor_by_name: Dict[str, Type] = {}
    field_by_predicate_local: Dict[Type, Dict[str, str]] = {}
    field_by_descriptor: Dict[Type, Dict[Type, str]] = {}

    for attr_name in dir(model_module):
        obj = getattr(model_module, attr_name)
        if isinstance(obj, type):
            # dataclasses that represent OWL classes have eq=False in this project; but safest is is_dataclass
            if is_dataclass(obj):
                class_by_name[attr_name] = obj
            # property descriptor classes inherit from PropertyDescriptor but we avoid importing it here
            # They are dataclasses too (frozen=True), but they should not have __annotations__ we care about
            # Use heuristic: they are dataclasses but do not have fields() (no fields) -> consider them descriptors
            try:
                flds = fields(obj)
            except TypeError:
                flds = ()
            if not flds and attr_name and attr_name[0].isupper():
                descriptor_by_name[attr_name] = obj

    # For each model class, map predicate local names to field names and descriptors to fields
    for cls_name, cls in list(class_by_name.items()):
        try:
            flds = fields(cls)
        except TypeError:
            continue
        pred_map: Dict[str, str] = {}
        desc_map: Dict[Type, str] = {}
        for f in flds:
            # Skip synthetic fields (no default) but handle both relation and datatype properties
            pred_local = f.name  # snake_case field name
            pred_map.setdefault(pred_local, f.name)
            # For relation fields, default value is an instance of a PropertyDescriptor subclass
            default_val = f.default
            if (
                default_val is not None
                and type(default_val).__name__ != "_MISSING_TYPE"
            ):
                # Some dataclasses may use default_factory embedded in descriptor; in our generated code
                # default is the descriptor instance
                desc_cls = type(default_val)
                if desc_cls is not None and desc_cls.__name__[0].isupper():
                    desc_map[desc_cls] = f.name
        field_by_predicate_local[cls] = pred_map
        field_by_descriptor[cls] = desc_map

    return (
        class_by_name,
        descriptor_by_name,
        field_by_predicate_local,
        field_by_descriptor,
    )


def _get_python_class_for_rdf_class(
    class_by_name: Dict[str, Type], rdf_class: URIRef
) -> Optional[Type]:
    name = local_name(rdf_class)
    # Expect PascalCase names in model equal to RDF local name
    return class_by_name.get(name)


def _coerce_literal(val: Literal, target_type: Optional[Type]) -> Any:
    if target_type is None:
        return val.toPython()
    try:
        # Unwrap Optional[T]
        origin = getattr(target_type, "__origin__", None)
        if origin is Union:
            args = [
                a for a in getattr(target_type, "__args__", ()) if a is not type(None)
            ]  # noqa: E721
            if args:
                target_type = args[0]
        if target_type in (str, int, float, bool):
            return target_type(val.toPython())
    except Exception:
        pass
    return val.toPython()


def load_instances(
    owl_path: str, model_module: Union[str, ModuleType]
) -> OwlInstancesRegistry:
    """Load OWL/RDF instances into the provided generated Python model module.

    This function is generic and can be reused with other OWL instance files that
    correspond to the given model module.
    """
    if isinstance(model_module, str):
        model_module = __import__(model_module, fromlist=["*"])
    g = rdflib.Graph()
    g.parse(owl_path)

    (
        class_by_name,
        descriptor_by_name,
        field_by_predicate_local,
        field_by_descriptor,
    ) = _collect_model_metadata(model_module)

    registry = OwlInstancesRegistry()

    # First, create all instances that have an explicit rdf:type matching our model
    for s, _, o_class in g.triples((None, RDF.type, None)):
        if not isinstance(s, URIRef):
            continue
        py_cls = _get_python_class_for_rdf_class(class_by_name, o_class)
        if py_cls is None:
            continue
        registry.get_or_create_for(s, py_cls)

    # Helper to ensure object instance exists by looking up its type dynamically
    def ensure_instance(uri: URIRef) -> Optional[Any]:
        inst = registry.resolve(uri)
        if inst is not None:
            return inst
        # Try to infer class from rdf:type triples
        for _, _, o_class in g.triples((uri, RDF.type, None)):
            py_cls = _get_python_class_for_rdf_class(class_by_name, o_class)
            if py_cls is not None:
                return registry.get_or_create_for(uri, py_cls)
        return None

    # For convenience: map property local name to descriptor base class (if exists)
    def descriptor_base_for(pred_local: str) -> Optional[Type]:
        return descriptor_by_name.get(to_pascal(pred_local))

    # Assign properties
    for s, p, o in g:
        if p == RDF.type:
            continue
        if not isinstance(s, URIRef):
            continue
        subj = registry.resolve(s)
        if subj is None:
            # Subject without explicit type known to model; try infer
            subj = ensure_instance(s)
            if subj is None:
                continue
        pred_local = local_name(p)
        snake = to_snake(pred_local)
        subj_cls = type(subj)
        # Determine the appropriate field name on the subject
        field_name = field_by_predicate_local.get(subj_cls, {}).get(snake)

        if isinstance(o, Literal):
            if field_name and hasattr(subj, field_name):
                # Coerce to field annotated type
                try:
                    ftypes = {f.name: f.type for f in fields(subj_cls)}
                except TypeError:
                    ftypes = {}
                coerced = _coerce_literal(o, ftypes.get(field_name))
                setattr(subj, field_name, coerced)
            # else: ignore literals not present in model
            continue

        # Object property
        obj = ensure_instance(o) if isinstance(o, URIRef) else None
        if field_name and hasattr(subj, field_name):
            lst = getattr(subj, field_name, None)
            if isinstance(lst, list) and obj is not None:
                lst.append(obj)
                continue
        # Try inverse assignment if direct field does not exist on subject
        base_desc = descriptor_base_for(pred_local)
        if base_desc is not None and obj is not None:
            # Find inverse_of on the base descriptor class (attribute may be missing)
            inverse = getattr(base_desc, "inverse_of", None)
            if inverse is not None:
                obj_cls = type(obj)
                inv_field = field_by_descriptor.get(obj_cls, {}).get(inverse)
                if inv_field and hasattr(obj, inv_field):
                    lst = getattr(obj, inv_field, None)
                    if isinstance(lst, list):
                        lst.append(subj)
                        continue
        # Fallback: if both sides have a list field with the same snake name, try assign on subject
        if (
            field_name
            and hasattr(subj, field_name)
            and isinstance(getattr(subj, field_name), list)
            and obj is not None
        ):
            getattr(subj, field_name).append(obj)

    return registry
