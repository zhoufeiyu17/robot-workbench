"""Disk-backed Infer results; bounded pages, no resident historical catalog.

The original v1 JSON is retained after a transactional, streaming import. Only
this business service opens the index; the control Owner never depends on it.
"""
from contextlib import closing
import json
from pathlib import Path
import sqlite3

DATABASE = 'history.sqlite3'
PAGE_RUNS = 12
PAGE_BYTES = 768 * 1024
ENTRY_BYTES = 256 * 1024
RUN_BYTES = 512 * 1024
RUN_RESULTS = 8
SCHEMA = 1


def _finite(value):
    raise ValueError('nonfinite inference history metadata: ' + value)


def legacy_entries(path):
    """Decode one old entry at a time, including pretty-printed UTF-8 JSON."""
    if Path(path).is_symlink() or not Path(path).is_file():raise ValueError('regular inference history JSON required')
    decoder = json.JSONDecoder(parse_constant=_finite)
    with Path(path).open(encoding='utf-8') as stream:
        buffer = ''; eof = False

        def peek():
            nonlocal buffer, eof
            while not buffer.strip() and not eof:
                buffer = stream.read(65536); eof = not buffer
            buffer = buffer.lstrip()
            return buffer[:1]

        def take(char):
            nonlocal buffer
            if peek() != char:raise ValueError('invalid inference history JSON delimiter')
            buffer = buffer[1:]

        def value():
            nonlocal buffer, eof
            peek()
            while True:
                try:
                    result, end = decoder.raw_decode(buffer)
                    if end > ENTRY_BYTES * 4:raise ValueError('inference history entry exceeds import limit')
                    buffer = buffer[end:]; return result
                except json.JSONDecodeError:
                    if eof:raise ValueError('incomplete inference history JSON') from None
                    if len(buffer) > ENTRY_BYTES * 4:raise ValueError('inference history entry exceeds import limit')
                    chunk = stream.read(65536); eof = not chunk; buffer += chunk

        take('{'); seen = set(); schema = None
        while peek() != '}':
            key = value()
            if not isinstance(key, str) or key in seen or key not in ('schema', 'entries'):
                raise ValueError('unknown or duplicate inference history field')
            seen.add(key); take(':')
            if key == 'schema':schema = value()
            else:
                take('[')
                if peek() != ']':
                    while True:
                        yield value()
                        if peek() == ']':break
                        take(',')
                take(']')
            if peek() == '}':break
            take(',')
            if peek() == '}':raise ValueError('trailing inference history field separator')
        take('}')
        if peek() or schema != 'inference_history.v1' or seen != {'schema', 'entries'}:
            raise ValueError('unknown inference history schema or trailing data')


def checked_entry(entry):
    if not isinstance(entry, dict) or not isinstance(entry.get('result'), dict):
        raise ValueError('invalid inference history entry')
    result = entry['result']; identity = result.get('id')
    media = result.get('media') or {}
    if not isinstance(media, dict):raise ValueError('invalid inference history media')
    run = result.get('run_id') or media.get('run_id') or identity
    if any(not isinstance(v, str) or not v or len(v) > 512 for v in (identity, run)):
        raise ValueError('bounded inference result and run IDs required')
    relative = entry.get('path')
    if relative is not None and (not isinstance(relative, str) or not relative or len(relative) > 4096
            or '\\' in relative or Path(relative).is_absolute() or '..' in Path(relative).parts
            or Path(relative).as_posix() != relative or Path(relative).suffix != '.mp4'):
        raise ValueError('inference history media path escapes output or is not an MP4')
    raw = json.dumps(result, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
    size = len(raw.encode())
    if size > ENTRY_BYTES:raise ValueError('inference result metadata exceeds 256 KiB; full evidence remains on disk')
    return identity, run, raw, relative, size


def _connect(path, *, readonly=False):
    path = Path(path)
    if path.is_symlink():raise ValueError('inference history database cannot be a symlink')
    db = sqlite3.connect(path.resolve().as_uri() + '?mode=ro', uri=True, timeout=5.) if readonly else sqlite3.connect(path, timeout=5.)
    try:
        db.execute('PRAGMA cache_size=-1024')
        db.execute('PRAGMA mmap_size=0')
    except Exception:
        db.close(); raise
    return db


def _legacy_pin(root):
    path = Path(root) / 'history.json'
    if not path.exists() and not path.is_symlink():return None
    if path.is_symlink() or not path.is_file():raise ValueError('regular inference history JSON required')
    from .collection_history import _pin
    return _pin(path)


def _check_legacy(db, root):
    row = db.execute("SELECT value FROM metadata WHERE key='legacy'").fetchone()
    if row is None:raise ValueError('incomplete inference history migration metadata')
    current = _legacy_pin(root)
    if current is not None and current != json.loads(row[0]):
        raise ValueError('legacy history.json changed after migration; preserve both indexes and reconcile before continuing')


def entries(root):
    """Read-only streaming enumeration for explicit preservation, never polling."""
    root = Path(root); path = root / DATABASE
    if path.exists() or path.is_symlink():
        with closing(_connect(path, readonly=True)) as db:
            if db.execute('PRAGMA user_version').fetchone()[0] != SCHEMA:
                raise ValueError('unknown or incomplete inference history database')
            if db.execute('PRAGMA journal_mode').fetchone()[0] != 'delete' or (root / (DATABASE + '-journal')).exists():
                raise ValueError('inference history must be idle with a standalone DELETE-journal index before preservation')
            _check_legacy(db, root)
            for raw, relative in db.execute('SELECT result, path FROM results ORDER BY seq'):
                yield dict(result=json.loads(raw, parse_constant=_finite), path=relative)
    elif (root / 'history.json').exists():
        yield from legacy_entries(root / 'history.json')


class InferenceHistory:
    def __init__(self, root):
        self.root = Path(root); self.path = self.root / DATABASE
        self.root.mkdir(parents=True, exist_ok=True)
        with closing(_connect(self.path)) as db, db:
            version = db.execute('PRAGMA user_version').fetchone()[0]
            if version == SCHEMA:
                _check_legacy(db, self.root); return
            if version != 0:raise ValueError('unknown inference history database version')
            # DELETE journals make an idle, closed index one independently
            # pinnable file for the existing maintenance preservation protocol.
            db.execute('PRAGMA journal_mode=DELETE')
            db.execute('BEGIN IMMEDIATE')
            if db.execute('PRAGMA user_version').fetchone()[0] == SCHEMA:
                _check_legacy(db, self.root); return
            db.execute('CREATE TABLE IF NOT EXISTS runs (seq INTEGER PRIMARY KEY, id TEXT UNIQUE NOT NULL, bytes INTEGER NOT NULL DEFAULT 0, count INTEGER NOT NULL DEFAULT 0)')
            db.execute('CREATE TABLE IF NOT EXISTS results (seq INTEGER PRIMARY KEY, id TEXT UNIQUE NOT NULL, run INTEGER NOT NULL REFERENCES runs(seq), result TEXT NOT NULL, path TEXT)')
            db.execute('CREATE INDEX IF NOT EXISTS results_run ON results(run, seq)')
            db.execute('CREATE TABLE IF NOT EXISTS metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL)')
            legacy = self.root / 'history.json'
            legacy_pin = _legacy_pin(self.root)
            if legacy.exists() or legacy.is_symlink():self._append(db, legacy_entries(legacy))
            if _legacy_pin(self.root) != legacy_pin:raise ValueError('legacy inference history changed during import')
            db.execute('INSERT INTO metadata(key,value) VALUES (?,?)', ('legacy', json.dumps(legacy_pin)))
            db.execute('PRAGMA user_version=1')

    @staticmethod
    def _append(db, rows):
        for entry in rows:
            identity, run, raw, relative, size = checked_entry(entry)
            db.execute('INSERT OR IGNORE INTO runs(id) VALUES (?)', (run,))
            seq, total, count = db.execute('SELECT seq, bytes, count FROM runs WHERE id=?', (run,)).fetchone()
            if total + size > RUN_BYTES or count >= RUN_RESULTS:
                raise ValueError('inference run index exceeds 512 KiB or 8 results; original evidence retained')
            db.execute('INSERT INTO results(id, run, result, path) VALUES (?,?,?,?)', (identity, seq, raw, relative))
            db.execute('UPDATE runs SET bytes=bytes+?, count=count+1 WHERE seq=?', (size, seq))

    def append(self, rows):
        with closing(_connect(self.path)) as db, db:
            self._append(db, (dict(result=row, path=str(path.relative_to(self.root)) if path is not None else None)
                              for row, path in rows))

    def page(self, *, before=None, after=None):
        if (before is not None and after is not None or any(type(v) is not int or not 0 < v < 2**63
                for v in (before, after) if v is not None)):
            raise ValueError('one positive inference history cursor required')
        with closing(_connect(self.path, readonly=True)) as db, db:
            db.execute('BEGIN')
            where, args, order = ('WHERE seq<?', (before,), 'DESC') if before is not None else (
                ('WHERE seq>?', (after,), 'ASC') if after is not None else ('', (), 'DESC'))
            candidates = db.execute('SELECT seq,id,bytes FROM runs '+where+' ORDER BY seq '+order+' LIMIT ?',
                                    (*args, PAGE_RUNS)).fetchall()
            selected = []; size = 0
            for row in candidates:
                if size + row[2] > PAGE_BYTES:break
                selected.append(row); size += row[2]
            selected.sort(reverse=True)
            ids = [row[0] for row in selected]
            rows = db.execute('SELECT result FROM results WHERE run IN ('+','.join('?' for _ in ids)+') ORDER BY seq', ids) if ids else []
            results = [json.loads(row[0], parse_constant=_finite) for row in rows]
            older = bool(ids and db.execute('SELECT 1 FROM runs WHERE seq<? LIMIT 1', (min(ids),)).fetchone())
            newer = bool(ids and db.execute('SELECT 1 FROM runs WHERE seq>? LIMIT 1', (max(ids),)).fetchone())
            return dict(schema='inference_results_page.v1', results=results, run_ids=[row[1] for row in selected],
                total_runs=db.execute('SELECT COUNT(*) FROM runs').fetchone()[0],
                total_results=db.execute('SELECT COUNT(*) FROM results').fetchone()[0],
                page_run_limit=PAGE_RUNS, page_byte_limit=PAGE_BYTES, result_bytes=size,
                older_cursor=min(ids) if older else None, newer_cursor=max(ids) if newer else None)

    def media(self, identity):
        if not isinstance(identity, str) or len(identity) > 512:return None
        with closing(_connect(self.path, readonly=True)) as db:
            row = db.execute('SELECT path FROM results WHERE id=?', (identity,)).fetchone()
        path = self.root / row[0] if row and row[0] else None
        return path if path and path.is_file() and not path.is_symlink() and path.resolve().is_relative_to(self.root.resolve()) else None
