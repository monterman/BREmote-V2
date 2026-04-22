#!/usr/bin/env node
'use strict';

const assert = require('node:assert');
const fs = require('node:fs');
const path = require('node:path');

// ═══════════════════════════════════════════════════════════════════
// USER CONFIGS — Paste your real device configs here for round-trip testing.
// Each entry: { device: 'tx'|'rx', version: '1'|'2'|'3', base64: '...' }
// ═══════════════════════════════════════════════════════════════════
const USER_CONFIGS = [
     { device: 'tx', version: '1', base64: 'AQABAAAAAQBkACQxjDcoQ1Q21yr0AR4A9AGIE9AHZADoAwoA0AcAAAEACgAKAAAAMgAyADIAMgBDrkI5AQAVWYgVWag=' },
     { device: 'rx', version: '1', base64: 'AQABAPf/AAAyAAAAAADoA9AH6APQB+gDAACgQQAASEIAAAAAAgAAAEGOHDwBABVZqBVZiA==' },
     { device: 'tx', version: '2', base64: 'AgABAAoAAQBkAHowbDcAQWI2syj0AR4A9AGIE9AHZADoAwoA0AcAAAEACgAKAAAAMgAAADIAAABDrkI5AAAAAAAAAADoAwEAFVmIFVmoAAA=' },
     { device: 'rx', version: '2', base64: 'AgABABQAAQBLAAEAAADoAwgH6AMIB+gDCgAAAAAAAAAAAAAAAAAAAAAAyEEAACBBAAAgQQAAoEAAAAxCAAA0QgAANEJBjhw8AAAAAOgDAQABABVcgBVZSA==' },



];
// ═══════════════════════════════════════════════════════════════════

// ── Extract and eval the converter logic from the HTML file ──
const htmlPath = path.join(__dirname, '..', 'config_converter.html');
const html = fs.readFileSync(htmlPath, 'utf-8');
const scriptMatch = html.match(/<script>([\s\S]*?)<\/script>/);
if (!scriptMatch) throw new Error('Could not extract <script> from config_converter.html');

let scriptCode = scriptMatch[1];
// Strip DOM-dependent functions and calls
// Remove entire function blocks that use DOM APIs
scriptCode = scriptCode
    .replace(/function renderFields\b[\s\S]*?\n\}/m, '')
    .replace(/function updateSizeLabels\b[\s\S]*?\n\}/m, '')
    .replace(/function updateVersionOptions\b[\s\S]*?\n\}/m, '')
    .replace(/function convert\b[\s\S]*?\n\}/m, '')
    .replace(/function copyOutput\b[\s\S]*?\n\}/m, '')
    // Remove any remaining lines with document/navigator/addEventListener
    .split('\n')
    .filter(line => !line.match(/\b(document|navigator|addEventListener|updateVersionOptions|updateSizeLabels)\s*\(/))
    .join('\n');
// Replace browser atob/btoa with Node equivalents
scriptCode = 'const atob = s => Buffer.from(s, "base64").toString("binary");\n'
           + 'const btoa = s => Buffer.from(s, "binary").toString("base64");\n'
           + scriptCode;

// Eval into a sandbox object
const fn = new Function(scriptCode + `
    return {
        U16, I16, FLOAT, ADDR3, STR8,
        computeLayout, fieldSize, fieldAlign, F,
        TX_V1_FIELDS, TX_V2_FIELDS, TX_V3_FIELDS,
        RX_V1_FIELDS, RX_V2_FIELDS, RX_V3_FIELDS,
        RENAMES, getFields,
        readField, writeField, b64Decode, b64Encode,
        addrToHex, hexToAddr,
        decodeStruct, encodeStruct, convertValues,
        formatValue, parseJsonInput, valuesToJson,
    };
`);
const C = fn();

// ── Test runner ──
let passed = 0, failed = 0, errors = [];
function test(name, fn) {
    try {
        fn();
        passed++;
        console.log(`  PASS  ${name}`);
    } catch (e) {
        failed++;
        errors.push({ name, error: e });
        console.log(`  FAIL  ${name}`);
        console.log(`        ${e.message}`);
    }
}

// ═══════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════

console.log('\n=== Struct Sizes ===');

test('TX V1 size = 68 bytes', () => {
    assert.strictEqual(C.computeLayout(C.TX_V1_FIELDS).totalSize, 68);
});
test('TX V2 size = 80 bytes', () => {
    assert.strictEqual(C.computeLayout(C.TX_V2_FIELDS).totalSize, 80);
});
test('TX V3 size > 80 bytes', () => {
    const size = C.computeLayout(C.TX_V3_FIELDS).totalSize;
    assert.ok(size > 80, `TX V3 should be > 80, got ${size}`);
});
test('RX V1 size = 52 bytes', () => {
    assert.strictEqual(C.computeLayout(C.RX_V1_FIELDS).totalSize, 52);
});
test('RX V2 size = 88 bytes', () => {
    assert.strictEqual(C.computeLayout(C.RX_V2_FIELDS).totalSize, 88);
});

console.log('\n=== Hex Address Formatting ===');

test('addrToHex formats [0,0,0] as 00:00:00', () => {
    assert.strictEqual(C.addrToHex([0, 0, 0]), '00:00:00');
});
test('addrToHex formats [171,205,239] as AB:CD:EF', () => {
    assert.strictEqual(C.addrToHex([171, 205, 239]), 'AB:CD:EF');
});
test('addrToHex formats [255,0,128] as FF:00:80', () => {
    assert.strictEqual(C.addrToHex([255, 0, 128]), 'FF:00:80');
});
test('hexToAddr parses AB:CD:EF to [171,205,239]', () => {
    assert.deepStrictEqual(C.hexToAddr('AB:CD:EF'), [171, 205, 239]);
});
test('hexToAddr parses 00:00:00 to [0,0,0]', () => {
    assert.deepStrictEqual(C.hexToAddr('00:00:00'), [0, 0, 0]);
});
test('hexToAddr parses lowercase ab:cd:ef', () => {
    assert.deepStrictEqual(C.hexToAddr('ab:cd:ef'), [171, 205, 239]);
});
test('hexToAddr rejects invalid format', () => {
    assert.throws(() => C.hexToAddr('ABCDEF'), /3 hex bytes/);
});
test('hexToAddr rejects invalid hex byte', () => {
    assert.throws(() => C.hexToAddr('GG:00:00'), /Invalid hex/);
});
test('addrToHex/hexToAddr round-trip', () => {
    const original = [0x1A, 0x2B, 0x3C];
    assert.deepStrictEqual(C.hexToAddr(C.addrToHex(original)), original);
});

console.log('\n=== formatValue uses hex for addresses ===');

test('formatValue ADDR3 shows hex', () => {
    const result = C.formatValue([171, 205, 239], C.ADDR3);
    assert.strictEqual(result, 'AB:CD:EF');
});

console.log('\n=== Round-Trip: Defaults (synthetic) ===');

const ALL_STRUCTS = [
    { name: 'TX V1', device: 'tx', version: '1', fields: C.TX_V1_FIELDS },
    { name: 'TX V2', device: 'tx', version: '2', fields: C.TX_V2_FIELDS },
    { name: 'TX V3', device: 'tx', version: '3', fields: C.TX_V3_FIELDS },
    { name: 'RX V1', device: 'rx', version: '1', fields: C.RX_V1_FIELDS },
    { name: 'RX V2', device: 'rx', version: '2', fields: C.RX_V2_FIELDS },
    { name: 'RX V3', device: 'rx', version: '3', fields: C.RX_V3_FIELDS },
];

for (const s of ALL_STRUCTS) {
    test(`${s.name} default encode->decode round-trip`, () => {
        const defaults = {};
        for (const f of s.fields) defaults[f.name] = f.default;
        const encoded = C.encodeStruct(defaults, s.fields);
        const decoded = C.decodeStruct(encoded, s.fields);
        for (const f of s.fields) {
            if (f.type === C.FLOAT) {
                assert.ok(Math.abs(decoded[f.name] - defaults[f.name]) < 1e-5,
                    `${f.name}: ${decoded[f.name]} != ${defaults[f.name]}`);
            } else if (f.type === C.ADDR3) {
                assert.deepStrictEqual(decoded[f.name], defaults[f.name], f.name);
            } else if (f.type === C.STR8) {
                assert.strictEqual(decoded[f.name].replace(/\0+$/, ''), defaults[f.name].replace(/\0+$/, ''), f.name);
            } else {
                assert.strictEqual(decoded[f.name], defaults[f.name], f.name);
            }
        }
    });

    test(`${s.name} default base64 round-trip`, () => {
        const defaults = {};
        for (const f of s.fields) defaults[f.name] = f.default;
        const buf1 = C.encodeStruct(defaults, s.fields);
        const b64 = C.b64Encode(buf1);
        const buf2 = C.b64Decode(b64);
        assert.deepStrictEqual(Array.from(buf2), Array.from(buf1));
    });
}

console.log('\n=== Round-Trip: Custom Address Values ===');

test('TX V2 with custom addresses round-trips', () => {
    const vals = {};
    for (const f of C.TX_V2_FIELDS) vals[f.name] = f.default;
    vals.own_address = [0xAA, 0xBB, 0xCC];
    vals.dest_address = [0x11, 0x22, 0x33];
    const buf = C.encodeStruct(vals, C.TX_V2_FIELDS);
    const decoded = C.decodeStruct(buf, C.TX_V2_FIELDS);
    assert.deepStrictEqual(decoded.own_address, [0xAA, 0xBB, 0xCC]);
    assert.deepStrictEqual(decoded.dest_address, [0x11, 0x22, 0x33]);
});

console.log('\n=== Version Conversion ===');

test('TX V1->V2: shared fields preserved, new fields get defaults', () => {
    const v1vals = {};
    for (const f of C.TX_V1_FIELDS) v1vals[f.name] = f.default;
    v1vals.max_gears = 8;
    v1vals.own_address = [0xAA, 0xBB, 0xCC];
    const { values, defaulted } = C.convertValues(v1vals, C.TX_V1_FIELDS, C.TX_V2_FIELDS);
    assert.strictEqual(values.max_gears, 8);
    assert.deepStrictEqual(values.own_address, [0xAA, 0xBB, 0xCC]);
    assert.strictEqual(values.version, 2);
    assert.strictEqual(values.gps_en, 0);
    assert.ok(defaulted.has('gps_en'));
});

test('TX V2->V1: removed fields dropped, shared preserved', () => {
    const v2vals = {};
    for (const f of C.TX_V2_FIELDS) v2vals[f.name] = f.default;
    v2vals.max_gears = 5;
    v2vals.gps_en = 1;
    const { values } = C.convertValues(v2vals, C.TX_V2_FIELDS, C.TX_V1_FIELDS);
    assert.strictEqual(values.max_gears, 5);
    assert.strictEqual(values.version, 1);
    assert.strictEqual(values.gps_en, undefined);
});

test('TX V2->V3: no_gear renamed to throttle_mode', () => {
    const v2vals = {};
    for (const f of C.TX_V2_FIELDS) v2vals[f.name] = f.default;
    v2vals.no_gear = 1;
    const { values } = C.convertValues(v2vals, C.TX_V2_FIELDS, C.TX_V3_FIELDS);
    assert.strictEqual(values.throttle_mode, 1);
    assert.strictEqual(values.version, 3);
});

test('TX V3->V2: throttle_mode renamed back to no_gear', () => {
    const v3vals = {};
    for (const f of C.TX_V3_FIELDS) v3vals[f.name] = f.default;
    v3vals.throttle_mode = 2;
    const { values } = C.convertValues(v3vals, C.TX_V3_FIELDS, C.TX_V2_FIELDS);
    assert.strictEqual(values.no_gear, 2);
    assert.strictEqual(values.version, 2);
});

test('TX V1->V3: multi-step conversion preserves fields', () => {
    const v1vals = {};
    for (const f of C.TX_V1_FIELDS) v1vals[f.name] = f.default;
    v1vals.max_gears = 7;
    v1vals.no_gear = 1;
    v1vals.own_address = [0xDE, 0xAD, 0x01];
    const { values } = C.convertValues(v1vals, C.TX_V1_FIELDS, C.TX_V3_FIELDS);
    assert.strictEqual(values.max_gears, 7);
    assert.strictEqual(values.throttle_mode, 1);
    assert.deepStrictEqual(values.own_address, [0xDE, 0xAD, 0x01]);
    assert.strictEqual(values.version, 3);
});

test('RX V1->V2: foil_bat fields dropped, foil_num_cells defaults', () => {
    const v1vals = {};
    for (const f of C.RX_V1_FIELDS) v1vals[f.name] = f.default;
    v1vals.failsafe_time = 2000;
    const { values, defaulted } = C.convertValues(v1vals, C.RX_V1_FIELDS, C.RX_V2_FIELDS);
    assert.strictEqual(values.failsafe_time, 2000);
    assert.strictEqual(values.foil_num_cells, 10);
    assert.ok(defaulted.has('foil_num_cells'));
    assert.strictEqual(values.version, 2);
});

test('RX V2->V1: foil_num_cells dropped, foil_bat fields default', () => {
    const v2vals = {};
    for (const f of C.RX_V2_FIELDS) v2vals[f.name] = f.default;
    v2vals.foil_num_cells = 14;
    const { values, defaulted } = C.convertValues(v2vals, C.RX_V2_FIELDS, C.RX_V1_FIELDS);
    assert.strictEqual(values.foil_bat_low, 10.0);
    assert.ok(defaulted.has('foil_bat_low'));
    assert.strictEqual(values.version, 1);
});

console.log('\n=== JSON Round-Trip with Hex Addresses ===');

test('valuesToJson outputs hex addresses', () => {
    const vals = {};
    for (const f of C.TX_V2_FIELDS) vals[f.name] = f.default;
    vals.own_address = [0xAB, 0xCD, 0xEF];
    const json = C.valuesToJson(vals, C.TX_V2_FIELDS);
    const obj = JSON.parse(json);
    assert.strictEqual(obj.own_address, 'AB:CD:EF');
});

test('parseJsonInput accepts hex address strings', () => {
    const json = JSON.stringify({ own_address: 'AB:CD:EF', dest_address: '11:22:33', version: 2 });
    const vals = C.parseJsonInput(json, C.TX_V2_FIELDS);
    assert.deepStrictEqual(vals.own_address, [0xAB, 0xCD, 0xEF]);
    assert.deepStrictEqual(vals.dest_address, [0x11, 0x22, 0x33]);
});

test('JSON encode->parse round-trip preserves addresses', () => {
    const vals = {};
    for (const f of C.TX_V2_FIELDS) vals[f.name] = f.default;
    vals.own_address = [0xDE, 0xAD, 0xBE];
    const json = C.valuesToJson(vals, C.TX_V2_FIELDS);
    const parsed = C.parseJsonInput(json, C.TX_V2_FIELDS);
    assert.deepStrictEqual(parsed.own_address, [0xDE, 0xAD, 0xBE]);
});

console.log('\n=== Full Pipeline: base64->decode->convert->encode->base64 ===');

test('TX V2 base64 -> V1 -> back to V2 preserves shared fields', () => {
    const original = {};
    for (const f of C.TX_V2_FIELDS) original[f.name] = f.default;
    original.max_gears = 6;
    original.thr_expo = 75;
    original.own_address = [0x12, 0x34, 0x56];
    original.dest_address = [0x78, 0x9A, 0xBC];
    const b64v2 = C.b64Encode(C.encodeStruct(original, C.TX_V2_FIELDS));

    const v2vals = C.decodeStruct(C.b64Decode(b64v2), C.TX_V2_FIELDS);

    const { values: v1vals } = C.convertValues(v2vals, C.TX_V2_FIELDS, C.TX_V1_FIELDS);
    const b64v1 = C.b64Encode(C.encodeStruct(v1vals, C.TX_V1_FIELDS));

    const v1decoded = C.decodeStruct(C.b64Decode(b64v1), C.TX_V1_FIELDS);
    const { values: v2again } = C.convertValues(v1decoded, C.TX_V1_FIELDS, C.TX_V2_FIELDS);

    assert.strictEqual(v2again.max_gears, 6);
    assert.strictEqual(v2again.thr_expo, 75);
    assert.deepStrictEqual(v2again.own_address, [0x12, 0x34, 0x56]);
    assert.deepStrictEqual(v2again.dest_address, [0x78, 0x9A, 0xBC]);
});

// ═══════════════════════════════════════════════════════════════════
// User Config Round-Trip Tests
// ═══════════════════════════════════════════════════════════════════
if (USER_CONFIGS.length > 0) {
    console.log('\n=== User Config Round-Trips ===');
    for (let i = 0; i < USER_CONFIGS.length; i++) {
        const uc = USER_CONFIGS[i];
        test(`User config #${i + 1} (${uc.device.toUpperCase()} V${uc.version}) round-trip`, () => {
            const fields = C.getFields(uc.device, uc.version);
            assert.ok(fields, `No struct for ${uc.device} V${uc.version}`);
            const bytes = C.b64Decode(uc.base64);
            const decoded = C.decodeStruct(bytes, fields);
            const reencoded = C.encodeStruct(decoded, fields);
            const b64out = C.b64Encode(reencoded);
            assert.strictEqual(b64out, uc.base64.trim(),
                `Round-trip mismatch for ${uc.device.toUpperCase()} V${uc.version}`);
        });
    }
} else {
    console.log('\n=== User Config Round-Trips ===');
    console.log('  SKIP  No USER_CONFIGS provided (add entries at top of file)');
}

// ── Summary ──
console.log(`\n${'='.repeat(50)}`);
console.log(`Results: ${passed} passed, ${failed} failed`);
if (errors.length > 0) {
    console.log('\nFailures:');
    for (const e of errors) {
        console.log(`  ${e.name}: ${e.error.message}`);
    }
    process.exit(1);
}
console.log('');
