# Map data

`osm/` holds OpenStreetMap downloads (buildings, woods and trees) cached by `src/swarm_tools/osm.py`.
With the cache, missions over the same area work offline and do not load the public Overpass servers
again. Each file is the Overpass API response to one query. Its name is the first 16 characters of the
SHA-1 hash of that query (area and filter), which is how the loader finds it again. The files can be
deleted at any time; they are downloaded again when needed.

Map data © OpenStreetMap contributors, available under the Open Database License (ODbL):
https://www.openstreetmap.org/copyright. Some map features carry public contact details that mappers
added to OpenStreetMap, such as a business e-mail address; they are part of the public map data.
