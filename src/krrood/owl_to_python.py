import os
import re
from typing import Dict, List

import rdflib
from jinja2 import Environment, FileSystemLoader
from rdflib.namespace import RDF, RDFS, OWL, XSD

from krrood import logger


class OwlToPythonConverter:
    def __init__(self, predefined_data_types: Dict[str, Dict[str, str]] | None = None):
        self.graph = rdflib.Graph()
        self.classes = {}
        self.properties = {}
        self.ontology_iri = ""
        self.predefined_data_types: Dict[str, Dict[str, str]] = (
            predefined_data_types or {}
        )

    def load_ontology(self, owl_file_path: str):
        """Load OWL file using RDFLib"""
        self.graph.parse(owl_file_path)
        self._extract_ontology_info()

    def _extract_ontology_info(self):
        """Extract classes, properties, and ontology metadata from ontology"""
        # Ontology name/label (used to create the ontology base class)
        ontology_label = None
        for onto in self.graph.subjects(RDF.type, OWL.Ontology):
            ontology_label = self._get_label(onto)
            if ontology_label:
                break
        self.ontology_label = ontology_label or "Ontology"

        # Extract classes
        for cls in self.graph.subjects(RDF.type, OWL.Class):
            class_info = self._extract_class_info(cls)
            self.classes[class_info["name"]] = class_info

        # Extract properties
        for prop in self.graph.subjects(RDF.type, OWL.ObjectProperty):
            prop_info = self._extract_property_info(prop)
            self.properties[prop_info["name"]] = prop_info

        for prop in self.graph.subjects(RDF.type, OWL.DatatypeProperty):
            prop_info = self._extract_property_info(prop)
            self.properties[prop_info["name"]] = prop_info

        # Include TransitiveProperty as (object) properties too
        for prop in self.graph.subjects(RDF.type, OWL.TransitiveProperty):
            prop_info = self._extract_property_info(prop)
            # If it already exists (also declared as ObjectProperty), merge flags
            existing = self.properties.get(prop_info["name"])
            if existing:
                existing["is_transitive"] = True
                # Preserve any previously computed inverse_of if missing
                if not existing.get("inverse_of"):
                    existing["inverse_of"] = prop_info.get("inverse_of")
                # Merge domains/ranges conservatively
                for k in (
                    "domains",
                    "ranges",
                    "range_uris",
                    "superproperties",
                    "inverses",
                ):
                    if k in prop_info:
                        existing[k] = sorted(
                            set(existing.get(k, [])) | set(prop_info.get(k, []))
                        )
            else:
                self.properties[prop_info["name"]] = prop_info

    def _extract_class_info(self, class_uri) -> Dict:
        """Extract information about a class

        - Prefer explicit rdfs:subClassOf values as base classes.
        - If none are present, include any named classes that appear in an owl:intersectionOf (ignore restrictions).
        """
        class_name = self._uri_to_python_name(class_uri)

        # Get superclasses from explicit rdfs:subClassOf
        superclasses: List[str] = []
        for superclass in self.graph.objects(class_uri, RDFS.subClassOf):
            if isinstance(superclass, rdflib.URIRef):
                superclasses.append(self._uri_to_python_name(superclass))

        # If no explicit superclasses, try owl:intersectionOf list items
        if not superclasses:
            for coll in self.graph.objects(class_uri, OWL.intersectionOf):
                # Traverse RDF list
                node = coll
                while node and node != RDF.nil:
                    first = self.graph.value(node, RDF.first)
                    if isinstance(first, rdflib.URIRef):
                        superclasses.append(self._uri_to_python_name(first))
                    # move to next
                    node = self.graph.value(node, RDF.rest)
                # only process first intersectionOf occurrence
                if superclasses:
                    break

        # De-duplicate while preserving order
        seen = set()
        unique_superclasses: List[str] = []
        for sc in superclasses:
            if sc not in seen:
                unique_superclasses.append(sc)
                seen.add(sc)

        # Get label
        label = self._get_label(class_uri)

        return {
            "name": class_name,
            "uri": str(class_uri),
            "superclasses": unique_superclasses or ["Thing"],
            "label": label,
            "comment": self._get_comment(class_uri),
        }

    def _extract_property_info(self, property_uri) -> Dict:
        """Extract information about a property"""
        prop_local = self._uri_to_python_name(property_uri)

        # Get domain and range
        domains: List[str] = []
        ranges: List[str] = []
        superproperties: List[str] = []
        inverses: List[str] = []

        for domain in self.graph.objects(property_uri, RDFS.domain):
            domains.append(self._uri_to_python_name(domain))

        range_uris: List[rdflib.term.Identifier] = []
        for range_val in self.graph.objects(property_uri, RDFS.range):
            ranges.append(self._uri_to_python_name(range_val))
            range_uris.append(range_val)

        # Inheritance between properties
        for super_prop in self.graph.objects(property_uri, RDFS.subPropertyOf):
            if isinstance(super_prop, rdflib.URIRef):
                superproperties.append(self._uri_to_python_name(super_prop))

        # Inverses
        for inv in self.graph.objects(property_uri, OWL.inverseOf):
            if isinstance(inv, rdflib.URIRef):
                inverses.append(self._uri_to_python_name(inv))
        # Also collect when current property is the object of inverseOf
        for inv_subj in self.graph.subjects(OWL.inverseOf, property_uri):
            if isinstance(inv_subj, rdflib.URIRef):
                inverses.append(self._uri_to_python_name(inv_subj))

        # Determine property type
        prop_type = "ObjectProperty"
        is_transitive = False
        for prop_type_uri in self.graph.objects(property_uri, RDF.type):
            if prop_type_uri == OWL.DatatypeProperty:
                prop_type = "DataProperty"
            if prop_type_uri == OWL.TransitiveProperty:
                is_transitive = True

        # Choose a single inverse if any (stable order)
        inverse_of = None
        if inverses:
            inverse_of = sorted(set(inverses))[0]

        return {
            "name": prop_local,
            "uri": str(property_uri),
            "type": prop_type,
            "domains": domains,
            "ranges": ranges,
            "range_uris": range_uris,
            "label": self._get_label(property_uri),
            "comment": self._get_comment(property_uri),
            "field_name": self._to_snake_case(prop_local),
            "descriptor_name": self._to_pascal_case(prop_local),
            "superproperties": superproperties,
            "inverses": sorted(set(inverses)),
            "inverse_of": inverse_of,
            "is_transitive": is_transitive,
        }

    def _uri_to_python_name(self, uri) -> str:
        """Convert URI to valid Python identifier"""
        if isinstance(uri, rdflib.URIRef):
            # Extract local name from URI
            uri_str = str(uri)
            if "#" in uri_str:
                local_name = uri_str.split("#")[-1]
            else:
                local_name = uri_str.split("/")[-1]

            # Convert to PascalCase for classes, camelCase for properties
            local_name = re.sub(r"[^a-zA-Z0-9_]", "_", local_name)
            return local_name
        return str(uri)

    def _to_snake_case(self, name: str) -> str:
        """Convert a name like 'worksFor' or 'WorksFor' to 'works_for'"""
        s1 = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
        s2 = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s1)
        return s2.lower()

    def _to_pascal_case(self, name: str) -> str:
        """Convert a name like 'worksFor' or 'works_for' to 'WorksFor'"""
        # If it contains underscores or hyphens, split and capitalize parts
        parts = re.split(r"[_\-\s]+", name)
        if len(parts) > 1:
            return "".join(p.capitalize() for p in parts if p)
        # Otherwise just capitalize first char
        return name[:1].upper() + name[1:]

    def _get_label(self, uri):
        """Get rdfs:label for a URI"""
        for label in self.graph.objects(uri, RDFS.label):
            return str(label)
        return None

    def _get_comment(self, uri):
        """Get rdfs:comment for a URI"""
        for comment in self.graph.objects(uri, RDFS.comment):
            return str(comment)
        return None

    def _topological_order(self, items: Dict[str, Dict], dep_key: str) -> List[str]:
        """Return a topological order based on dependency names in dep_key; if cycles, append remaining alphabetically."""
        remaining = {
            name: set(items[name].get(dep_key, [])) & set(items.keys())
            for name in items
        }
        ordered: List[str] = []
        while remaining:
            ready = sorted([name for name, deps in remaining.items() if not deps])
            if not ready:
                ordered.extend(sorted(remaining.keys()))
                break
            for name in ready:
                ordered.append(name)
                del remaining[name]
            for deps in remaining.values():
                deps.difference_update(ready)
        return ordered

    def generate_python_code_external(self) -> str:
        """Generate Python code using the external Jinja2 template with proper class/property inheritance.

        - Object properties: List[Range] where Range is a class name, or Union[...] if multiple ranges
        - Data properties: Optional[Type] where Type maps from XSD to Python types (Union if multiple ranges)
        - Avoid duplicate attributes in subclasses when already defined in ancestors
        """
        # Prepare class bases
        classes_copy: Dict[str, Dict] = {
            name: dict(info) for name, info in self.classes.items()
        }
        for info in classes_copy.values():
            info["base_classes"] = [
                b for b in info.get("superclasses", []) if b != "Thing"
            ]
        # Create ontology base class name from ontology label
        ontology_base_class_name = self._to_pascal_case(
            re.sub(r"\W+", " ", getattr(self, "ontology_label", "Ontology")).strip()
        )
        if not ontology_base_class_name.endswith("Ontology"):
            ontology_base_class_name = ontology_base_class_name + "Ontology"
        # Create synthetic ontology base class entry if not exists
        if ontology_base_class_name not in classes_copy:
            classes_copy[ontology_base_class_name] = {
                "name": ontology_base_class_name,
                "uri": "",
                "superclasses": ["Thing"],
                "base_classes": [],
                "label": f"Base class for {getattr(self, 'ontology_label', 'Ontology')}",
                "comment": None,
            }
        # Redirect root classes (no explicit base) to inherit from the ontology base (not Thing)
        for name, info in classes_copy.items():
            if name == ontology_base_class_name:
                continue
            bases = info.get("base_classes", [])
            if len(bases) == 0:
                info["base_classes"] = [ontology_base_class_name]
        # Determine decorator/metaclass application flags: only on original root classes (kept for compatibility)
        for name, info in classes_copy.items():
            bases = info.get("base_classes", [])
            is_root = name == ontology_base_class_name
            info["define_metaclass"] = is_root
            info["apply_symbol"] = is_root

        # Compute full ancestor sets for each class (transitive closure)
        name_to_bases = {
            name: set(info["base_classes"]) for name, info in classes_copy.items()
        }
        for name, info in classes_copy.items():
            ancestors = set()
            stack = list(info["base_classes"])
            while stack:
                base = stack.pop()
                if base in ancestors:
                    continue
                ancestors.add(base)
                stack.extend(name_to_bases.get(base, []))
            info["all_base_classes"] = sorted(ancestors)

        # Prepare property descriptor bases and compute type-hint helpers
        properties_copy: Dict[str, Dict] = {
            name: dict(info) for name, info in self.properties.items()
        }

        # Infer domains and ranges using subPropertyOf, inverseOf, and restrictions
        # Initialize maps
        dom_map = {
            name: set(info.get("domains", [])) for name, info in properties_copy.items()
        }
        rng_map = {
            name: set(info.get("ranges", [])) for name, info in properties_copy.items()
        }
        rng_uri_map = {
            name: set(info.get("range_uris", []))
            for name, info in properties_copy.items()
        }
        type_map = {
            name: info.get("type", "ObjectProperty")
            for name, info in properties_copy.items()
        }
        super_map = {
            name: list(info.get("superproperties", []))
            for name, info in properties_copy.items()
        }
        inverse_pairs = []
        for name, info in properties_copy.items():
            for inv in info.get("inverses", []) or []:
                if inv in properties_copy:
                    inverse_pairs.append((name, inv))

        # Restriction parser helpers
        property_restrictions: Dict[str, Dict[str, set]] = {}

        def _handle_restriction(for_class: str, node):
            if not node:
                return
            on_prop = self.graph.value(node, OWL.onProperty)
            if not on_prop:
                return
            prop_name = self._uri_to_python_name(on_prop)
            if prop_name not in properties_copy:
                return
            dom_map[prop_name].add(for_class)
            some = self.graph.value(node, OWL.someValuesFrom) or self.graph.value(
                node, OWL.allValuesFrom
            )
            if some:
                try:
                    rng_name = self._uri_to_python_name(some)
                    rng_map[prop_name].add(rng_name)
                    rng_uri_map[prop_name].add(some)
                    # Track per-class restriction to specialize properties later
                    cdict = property_restrictions.setdefault(for_class, {})
                    s = cdict.setdefault(prop_name, set())
                    s.add(rng_name)
                except Exception:
                    pass

        # Track declared domains that originate explicitly (rdfs:domain) or via class restrictions only
        declared_dom_map = {
            name: set(info.get("domains", [])) for name, info in properties_copy.items()
        }

        # Walk class restrictions
        for cls_uri in self.graph.subjects(RDF.type, OWL.Class):
            cls_name = self._uri_to_python_name(cls_uri)
            # direct subclass restrictions
            for restr in self.graph.objects(cls_uri, RDFS.subClassOf):
                _handle_restriction(cls_name, restr)
                # If restriction mentions a property, count this class as declared domain for that property
                on_prop = self.graph.value(restr, OWL.onProperty)
                if on_prop:
                    declared_dom_map[self._uri_to_python_name(on_prop)].add(cls_name)
            # restrictions inside intersectionOf
            for coll in self.graph.objects(cls_uri, OWL.intersectionOf):
                node = coll
                while node and node != RDF.nil:
                    first = self.graph.value(node, RDF.first)
                    _handle_restriction(cls_name, first)
                    on_prop = self.graph.value(first, OWL.onProperty) if first else None
                    if on_prop:
                        declared_dom_map[self._uri_to_python_name(on_prop)].add(
                            cls_name
                        )
                    node = self.graph.value(node, RDF.rest)
        # Fixed-point propagate via subPropertyOf and inverseOf (for types/ranges), but do NOT add to declared domains
        changed = True
        while changed:
            changed = False
            for name, supers in super_map.items():
                for sp in supers:
                    if sp not in dom_map:
                        continue
                    before_d, before_r, before_ru = (
                        len(dom_map[name]),
                        len(rng_map[name]),
                        len(rng_uri_map[name]),
                    )
                    dom_map[name].update(dom_map.get(sp, set()))
                    rng_map[name].update(rng_map.get(sp, set()))
                    rng_uri_map[name].update(rng_uri_map.get(sp, set()))
                    if (
                        len(dom_map[name]) != before_d
                        or len(rng_map[name]) != before_r
                        or len(rng_uri_map[name]) != before_ru
                    ):
                        changed = True
            for a, b in inverse_pairs:
                # a inverseOf b
                before_da, before_ra = len(dom_map[a]), len(rng_map[a])
                before_db, before_rb = len(dom_map[b]), len(rng_map[b])
                # Swap domains and ranges between inverses for type inference only
                dom_map[a].update(rng_map.get(b, set()))
                rng_map[a].update(dom_map.get(b, set()))
                dom_map[b].update(rng_map.get(a, set()))
                rng_map[b].update(dom_map.get(a, set()))
                if (
                    len(dom_map[a]) != before_da
                    or len(rng_map[a]) != before_ra
                    or len(dom_map[b]) != before_db
                    or len(rng_map[b]) != before_rb
                ):
                    changed = True
        # Do NOT generalize domains to ancestors: keep properties on the most specific classes that introduce them
        # Write back inferred domains/ranges
        for name, info in properties_copy.items():
            info["domains"] = sorted(dom_map.get(name, set()))
            info["ranges"] = sorted(rng_map.get(name, set()))
            info["range_uris"] = list(rng_uri_map.get(name, set()))
            info["declared_domains"] = sorted(declared_dom_map.get(name, set()))

        # Create specialized properties for class-specific range restrictions (object properties only)
        specialized_props: Dict[str, Dict] = {}
        for cls_name, props in property_restrictions.items():
            for prop_name, rng_names in props.items():
                base = properties_copy.get(prop_name)
                if not base or base.get("type") != "ObjectProperty":
                    continue
                # Remove this class from the base property's declared domains (we will attach a specialized one)
                base_dd = list(base.get("declared_domains", []))
                if cls_name in base_dd:
                    base_dd.remove(cls_name)
                    base["declared_domains"] = base_dd
                for rng_name in sorted(rng_names):
                    spec_key = f"{prop_name}__{rng_name}"
                    if spec_key in properties_copy or spec_key in specialized_props:
                        continue
                    spec = {
                        "name": spec_key,
                        "uri": base.get("uri", ""),
                        "type": "ObjectProperty",
                        "domains": [cls_name],
                        "ranges": [rng_name],
                        "range_uris": [],
                        "label": base.get("label"),
                        "comment": base.get("comment"),
                        "field_name": base.get("field_name"),
                        "descriptor_name": self._to_pascal_case(
                            base.get("descriptor_name", prop_name)
                        )
                        + self._to_pascal_case(rng_name),
                        "superproperties": [prop_name],
                        "inverses": [],
                        "inverse_of": None,
                        "is_transitive": base.get("is_transitive", False),
                        "declared_domains": [cls_name],
                    }
                    specialized_props[spec_key] = spec

        # Merge specialized properties
        properties_copy.update(specialized_props)

        # Attach datatype properties without an explicit domain to the ontology base class
        for name, info in properties_copy.items():
            if info.get("type") == "DataProperty" and not info.get("declared_domains"):
                info["declared_domains"] = [ontology_base_class_name]

        # Apply predefined data type overrides: class -> { property_snake -> python_type }
        # This may also coerce an object property into a data property and attach it to the class.
        for cls_name, overrides in (self.predefined_data_types or {}).items():
            for field_snake, py_type in overrides.items():
                # Find property by snake_case field name
                target_prop_name = None
                for prop_name, p in properties_copy.items():
                    if p.get("field_name") == field_snake:
                        target_prop_name = prop_name
                        break
                if not target_prop_name:
                    logger.info(
                        f"[owl_to_python] Override not applied: property '{field_snake}' not found"
                    )
                    continue
                p = properties_copy[target_prop_name]
                p["type"] = "DataProperty"
                p["data_type_hint_inner"] = py_type
                p["_predefined_data_type"] = True
                # track per-class overrides for correct emission precedence
                ov = set(p.get("_overrides_for", []))
                ov.add(cls_name)
                p["_overrides_for"] = sorted(ov)
                # ensure declared on the class
                dd = list(p.get("declared_domains", []))
                if cls_name not in dd:
                    dd.append(cls_name)
                p["declared_domains"] = dd
                logger.info(
                    f"[owl_to_python] Applied override: {cls_name}.{field_snake} -> {py_type}"
                )

        # XSD to Python mapping
        xsd_to_py = {
            XSD.string: "str",
            XSD.normalizedString: "str",
            XSD.token: "str",
            XSD.language: "str",
            XSD.boolean: "bool",
            XSD.decimal: "float",
            XSD.float: "float",
            XSD.double: "float",
            XSD.integer: "int",
            XSD.nonPositiveInteger: "int",
            XSD.negativeInteger: "int",
            XSD.long: "int",
            XSD.int: "int",
            XSD.short: "int",
            XSD.byte: "int",
            XSD.nonNegativeInteger: "int",
            XSD.unsignedLong: "int",
            XSD.unsignedInt: "int",
            XSD.unsignedShort: "int",
            XSD.unsignedByte: "int",
            XSD.positiveInteger: "int",
            XSD.date: "str",
            XSD.dateTime: "str",
            XSD.time: "str",
            XSD.anyURI: "str",
        }

        # Simplify object property ranges: remove subclasses if their ancestor is present
        ancestors_map = {
            name: set(info["all_base_classes"]) for name, info in classes_copy.items()
        }

        for name, info in properties_copy.items():
            # Base descriptors from superproperties
            bases: List[str] = []
            for sp in info.get("superproperties", []):
                if sp in properties_copy:
                    bases.append(properties_copy[sp]["descriptor_name"])
            if not bases:
                bases.append("PropertyDescriptor")
            info["base_descriptors"] = bases

            # Object vs data type hints
            if info["type"] == "ObjectProperty":
                ranges = list(info.get("ranges", []))
                if ranges:
                    rng_set = set(ranges)
                    simplified = []
                    for r in sorted(rng_set):
                        # If any ancestor of r is also in rng_set, skip r
                        r_ancestors = ancestors_map.get(r, set())
                        if any(a in rng_set for a in r_ancestors):
                            continue
                        simplified.append(r)
                    ranges = simplified or ranges
                # If multiple ranges, form a Union
                if len(ranges) > 1:
                    info["object_range_hint"] = (
                        "Union[" + ", ".join(sorted(set(ranges))) + "]"
                    )
                elif len(ranges) == 1:
                    info["object_range_hint"] = ranges[0]
                else:
                    # No range information available; warn and fall back to Any
                    logger.warning(
                        f"[owl_to_python]: Could not infer object range type for property '{name}'. Using Any."
                    )
                    info["object_range_hint"] = "Any"
            else:
                # Data property: map XSD ranges to Python types, unless predefined by user
                if info.get("_predefined_data_type") and info.get(
                    "data_type_hint_inner"
                ):
                    # Respect user-provided override
                    continue
                py_types: List[str] = []
                for uri in info.get("range_uris", []) or []:
                    try:
                        if isinstance(uri, rdflib.URIRef) and uri in xsd_to_py:
                            py_types.append(xsd_to_py[uri])
                    except Exception:
                        pass
                if not py_types:
                    # Fall back to mapping the textual names we already have
                    textual = [r.lower() for r in info.get("ranges", [])]
                    for t in textual:
                        if t in (
                            "string",
                            "normalizedstring",
                            "token",
                            "language",
                            "anyuri",
                            "datetime",
                            "date",
                            "time",
                        ):
                            py_types.append("str")
                        elif t in (
                            "integer",
                            "int",
                            "long",
                            "short",
                            "byte",
                            "nonnegativeinteger",
                            "positiveinteger",
                            "unsignedlong",
                            "unsignedint",
                            "unsignedshort",
                            "unsignedbyte",
                        ):
                            py_types.append("int")
                        elif t in ("float", "double", "decimal"):
                            py_types.append("float")
                        elif t in ("boolean",):
                            py_types.append("bool")
                    if not py_types:
                        # Could not determine type from ontology; fallback to Any with a warning
                        logger.warning(
                            f"[owl_to_python]: Could not infer data type for property '{name}'. Using Any."
                        )
                        py_types.append("Any")
                # Deduplicate while preserving order
                seen = set()
                py_types_unique = []
                for t in py_types:
                    if t not in seen:
                        py_types_unique.append(t)
                        seen.add(t)
                if len(py_types_unique) > 1:
                    info["data_type_hint_inner"] = (
                        "Union[" + ", ".join(py_types_unique) + "]"
                    )
                else:
                    info["data_type_hint_inner"] = py_types_unique[0]

        # Decide which properties to declare on each class (avoid duplicates)
        for cls_name, cls_info in classes_copy.items():
            ancestors = set(cls_info.get("all_base_classes", []))
            declared: List[str] = []
            for prop_name, p in properties_copy.items():
                declared_domains = p.get("declared_domains", [])
                applies_to_cls = cls_name in declared_domains
                if not applies_to_cls:
                    continue
                # If any ancestor is also a declared domain, skip on this class
                # EXCEPT when a predefined override explicitly targets this class.
                overrides_for = set(p.get("_overrides_for", []))
                skip = False
                if ancestors and cls_name not in overrides_for:
                    for a in ancestors:
                        if a in declared_domains:
                            skip = True
                            break
                if not skip:
                    declared.append(prop_name)
            # Compatibility: ensure 'headOf' attribute exists on Employee to keep MemberOf queries safe
            if cls_name == "Employee":
                if any(
                    pinfo.get("name") == "headOf" for pinfo in properties_copy.values()
                ):
                    if "headOf" not in declared:
                        declared.append("headOf")
                        declared = sorted(declared)
            cls_info["declared_properties"] = declared

        # Start with base-class-only topological order
        classes_order = self._topological_order(classes_copy, dep_key="base_classes")

        # Note: we deliberately do not reorder by object-range dependencies to avoid
        # violating base-class ordering and creating oscillations. Forward references
        # in type hints are handled by Thing/PropertyDescriptor patches.

        properties_order = self._topological_order(
            properties_copy, dep_key="superproperties"
        )

        # Precompute whether inverse target is defined prior in the descriptor order
        index_map = {name: idx for idx, name in enumerate(properties_order)}
        for name, info in properties_copy.items():
            inv = info.get("inverse_of")
            prior = False
            if inv and inv in properties_copy:
                prior = index_map.get(inv, 10**9) < index_map.get(name, 10**9)
            info["inverse_target_is_prior"] = prior

        template_dir = os.path.dirname(__file__)
        env = Environment(
            loader=FileSystemLoader(template_dir),
            autoescape=False,
            trim_blocks=True,
            lstrip_blocks=True,
        )
        template = env.get_template("jinja_template.j2")
        return template.render(
            classes=classes_copy,
            properties=properties_copy,
            classes_order=classes_order,
            properties_order=properties_order,
            ontology_base_class_name=ontology_base_class_name,
        )

    def save_to_file(self, output_path: str):
        """Generate and save Python code to file"""
        python_code = self.generate_python_code_external()
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(python_code)
        logger.info(f"Generated Python classes saved to: {output_path}")


# Usage
if __name__ == "__main__":
    from krrood.helpers import generate_lubm_with_predicates

    generate_lubm_with_predicates()
