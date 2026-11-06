/* Enumerate every stream inside a modern SolidWorks file using the reverse-engineered container.
 *
 * Usage (Node 18+, no npm install needed):
 *   node cad_geometry/extract_asm_stream.js CAD_model/full_assembly.SLDASM <outdir>
 * Then copy <outdir>/swXmlContents_COMPINSTANCETREE.bin to
 * cad_geometry/COMPINSTANCETREE.xml and run parse_cad_assembly.py.
 *
 * The decoder (Apache-2.0, github.com/blussyya/solidworks-converters) is
 * vendored in tools/sldprt so this works offline, with no SolidWorks install.
 */
const fs = require('fs');
const zlib = require('zlib');
const path = require('path');
const ROOT = path.join(__dirname, '..', 'tools', 'sldprt');
const modern = require(path.join(ROOT, 'src', 'container', 'modern.js'));
const MAX = 1024 * 1024 * 1024;
const inflateRaw = b => zlib.inflateRawSync(b, { maxOutputLength: MAX });
const inflateZlib = b => zlib.inflateSync(b, { maxOutputLength: MAX });
const file = process.argv[2];
const outDir = process.argv[3];
const buf = fs.readFileSync(file);
const streams = modern.decompressOpenSX(buf, inflateRaw, inflateZlib);
const names = Object.keys(streams).sort();
console.log('FILE', path.basename(file), 'streams', names.length);
for (const n of names) {
  const d = streams[n];
  console.log(String(d.length).padStart(10), n);
}
if (outDir) {
  fs.mkdirSync(outDir, { recursive: true });
  for (const n of names) {
    const safe = n.replace(/[^A-Za-z0-9_.-]/g, '_');
    fs.writeFileSync(path.join(outDir, safe + '.bin'), streams[n]);
  }
}
