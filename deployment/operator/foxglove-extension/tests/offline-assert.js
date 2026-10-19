// Small assertion adapter for running the existing pure model tests in GJS.
let testCount=0;
function fail(text){throw new Error(text);}
const assert={
  equal(actual,expected,why=''){if(!Object.is(actual,expected))fail(`${why}: ${String(actual)} !== ${String(expected)}`);},
  notEqual(actual,expected){if(Object.is(actual,expected))fail('Expected distinct values');},
  deepEqual(actual,expected){if(JSON.stringify(actual)!==JSON.stringify(expected))fail(`${JSON.stringify(actual)} != ${JSON.stringify(expected)}`);},
  ok(value){if(!value)fail('Expected truthy value');},
  match(value,pattern){if(!pattern.test(value))fail(`${value} does not match ${pattern}`);},
  throws(fn,pattern){let caught=false;try{fn();}catch(error){caught=true;if(pattern&&!pattern.test(String(error)))throw error;}if(!caught)fail('Expected an error');},
  doesNotThrow(fn){fn();},
};
const test=(name,fn)=>{fn();testCount++;print(`ok ${testCount} - ${name}`);};
